#!/usr/bin/env python3
"""openpilot supercombo ONNX 오프라인 추론 — DSS 카메라 프레임 차선 인식 PoC.

캡처된 프레임 시퀀스(capture_frames.py 산출)를 openpilot 등가 전처리(워프→YUV420
6채널 패킹)로 모델에 넣고 lane_lines 를 파싱해 원본 영상 위 오버레이 PNG 를 만든다.

openpilot 등가 재구현의 1차 소스 (commaai/openpilot@053d9c4, references/commaai/openpilot/):
  - 워프 행렬:  common/transformations/model.py get_warp_matrix (L65-70)
  - intrinsics: common/transformations/model.py L10-40 (med fl=910, sbig fl=455)
  - 축 치환:    common/transformations/camera.py (device x전방/y우/z하 → view x우/y하/z전방)
  - YUV 변환:   tools/sim/lib/camerad.py rgb_to_nv12 (BT.601 정수 계수)
  - 6채널 패킹: selfdrive/modeld/compile_modeld.py frames_to_tensor (L83-92)
  - 입력 큐:    compile_modeld.py — img 페어는 0.2 s(5 Hz) 간격, features 24스텝(5 Hz),
                DSS 실측 5.2~7.2 Hz ≈ 모델 문맥 5 Hz 라 연속 프레임을 그대로 페어로 쓴다
  - 출력 파싱:  parse_model_outputs.py parse_mdn / ONNX 메타데이터 output_slices
  - 확률 채움:  fill_model_msg.py L107 (sigmoid 후 [1::2])

함수표: docs/code_review/openpilot_lane/2026-08-17.md
"""
import argparse
import glob
import json
import math
import os

import cv2
import numpy as np
import onnxruntime as ort

_HERE = os.path.dirname(os.path.abspath(__file__))
# src/AI/openpilot_lane → 프로젝트 루트의 references/ (외부 1차 소스 보관 규약 경로)
_REF = os.path.normpath(os.path.join(_HERE, '..', '..', '..', 'references', 'commaai', 'openpilot'))
MODEL_PATH = os.path.join(_REF, 'driving_supercombo.onnx')

with open(os.path.join(_REF, 'output_slices.json')) as _f:
    SLICES = {k: slice(a, b) for k, (a, b) in json.load(_f).items()}

# constants.py index_function 등가: 33지점, 0~192 m 제곱 간격
X_IDXS = np.array([192.0 * (i / 32.0) ** 2 for i in range(33)], dtype=np.float32)

MODEL_W, MODEL_H = 512, 256
MEDMODEL_CY = 47.6
MEDMODEL_K = np.array([[910.0, 0.0, 256.0], [0.0, 910.0, MEDMODEL_CY], [0.0, 0.0, 1.0]])
SBIGMODEL_K = np.array([[455.0, 0.0, 256.0], [0.0, 455.0, 0.5 * (256 + MEDMODEL_CY)], [0.0, 0.0, 1.0]])

# device(x전방, y우, z하) → view(x우, y하, z전방)
VIEW_FROM_DEVICE = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])

# 좌외곽·좌차선·우차선·우외곽 (BGR)
LANE_COLORS = [(255, 128, 0), (0, 255, 0), (0, 255, 255), (0, 128, 255)]
EDGE_COLOR = (0, 0, 255)

DEFAULT_FOV_DEG = 90.0   # ⚠추정 — DSS CameraInfo 부재, 오버레이 시각 튜닝 대상


def rot_from_euler(euler):
    r, p, y = euler
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def get_warp_matrix(euler, cam_K, bigmodel_frame=False):
    """model.py get_warp_matrix 등가 — 모델 입력 픽셀 → 카메라 픽셀 사상(3×3)."""
    model_K = SBIGMODEL_K if bigmodel_frame else MEDMODEL_K
    calib_from_model = np.linalg.inv(model_K @ VIEW_FROM_DEVICE)
    camera_from_calib = cam_K @ VIEW_FROM_DEVICE @ rot_from_euler(euler)
    return camera_from_calib @ calib_from_model


def warp_to_model(rgb, M):
    """RGB 도메인 워프(openpilot 은 YUV 도메인 — 기하 동일, 보간 차이만)."""
    return cv2.warpPerspective(rgb, M.astype(np.float64), (MODEL_W, MODEL_H),
                               flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)


def rgb_to_yuv6ch(rgb):
    """camerad rgb_to_nv12 등가(BT.601 정수 계수) + frames_to_tensor 등가 패킹 → (6,128,256)."""
    r = rgb[:, :, 0].astype(np.int32)
    g = rgb[:, :, 1].astype(np.int32)
    b = rgb[:, :, 2].astype(np.int32)
    y = np.clip((((b * 13 + g * 65 + r * 33) + 64) >> 7) + 16, 0, 255).astype(np.uint8)
    r_s = (r[0::2, 0::2] + r[0::2, 1::2] + r[1::2, 0::2] + r[1::2, 1::2] + 2) >> 2
    g_s = (g[0::2, 0::2] + g[0::2, 1::2] + g[1::2, 0::2] + g[1::2, 1::2] + 2) >> 2
    b_s = (b[0::2, 0::2] + b[0::2, 1::2] + b[1::2, 0::2] + b[1::2, 1::2] + 2) >> 2
    u = np.clip((b_s * 56 - g_s * 37 - r_s * 19 + 0x8080) >> 8, 0, 255).astype(np.uint8)
    v = np.clip((r_s * 56 - g_s * 47 - b_s * 9 + 0x8080) >> 8, 0, 255).astype(np.uint8)
    # frames_to_tensor 채널 순서: Y[짝행,짝열] Y[홀행,짝열] Y[짝행,홀열] Y[홀행,홀열] U V
    return np.stack([y[0::2, 0::2], y[1::2, 0::2], y[0::2, 1::2], y[1::2, 1::2], u, v])


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def parse_mdn_lanes(vec):
    """lane_lines/road_edges: 앞 절반 mean → (N,33,2)=(y,z). 확률은 sigmoid 후 [1::2]."""
    ll = vec[SLICES['lane_lines']]
    lanes = ll[:len(ll) // 2].reshape(4, 33, 2)
    prob = sigmoid(vec[SLICES['lane_lines_prob']])[1::2]
    re = vec[SLICES['road_edges']]
    edges = re[:len(re) // 2].reshape(2, 33, 2)
    return lanes, prob, edges


def parse_plan(vec):
    """plan: 앞 절반 mean → (33,15), 첫 3열이 위치(x,y,z) — 주행 경로."""
    pl = vec[SLICES['plan']]
    return pl[:len(pl) // 2].reshape(33, 15)[:, :3]


def project_points(pts_calib, cam_K, euler, cam_height=0.0):
    """calib 3D → 카메라 픽셀. 실측 확정(2026-08-17 합성 검증): 모델 출력 좌표계는
    x전방·y우(+)·z하(+), 원점=카메라(차선 z≈+1.33 = 노면까지 거리) — 부호 반전·높이
    병진 불요. cam_height 는 예외적 미세조정용 view y 병진(기본 0)."""
    p = pts_calib.astype(np.float64)
    view = (VIEW_FROM_DEVICE @ rot_from_euler(euler) @ p.T).T
    view[:, 1] += cam_height
    valid = view[:, 2] > 0.5
    px = np.full((len(p), 2), np.nan)
    vz = view[valid]
    px[valid] = (cam_K @ (vz.T / vz[:, 2])).T[:, :2]
    return px, valid


def draw_overlay(bgr, lanes, prob, edges, cam_K, euler, cam_height=0.0, prob_thr=0.3,
                 plan=None, path_width=0.9):
    h, w = bgr.shape[:2]
    out = bgr.copy()
    # comma UI 스타일 주행 경로 밴드(반투명 초록) — plan 좌우 ±path_width
    if plan is not None:
        left = plan.copy(); left[:, 1] -= path_width
        right = plan.copy(); right[:, 1] += path_width
        pl, vl = project_points(left, cam_K, euler, cam_height)
        pr, vr = project_points(right, cam_K, euler, cam_height)
        valid = vl & vr
        if valid.sum() >= 2:
            poly = np.concatenate([pl[valid], pr[valid][::-1]]).astype(np.int32)
            layer = out.copy()
            cv2.fillPoly(layer, [poly], (80, 220, 80))
            out = cv2.addWeighted(layer, 0.35, out, 0.65, 0)
    for i in range(4):
        if prob[i] < prob_thr:
            continue
        pts3 = np.stack([X_IDXS, lanes[i, :, 0], lanes[i, :, 1]], axis=1)
        px, valid = project_points(pts3, cam_K, euler, cam_height)
        pix = px[valid].astype(np.int32)
        inside = (pix[:, 0] >= -w) & (pix[:, 0] < 2 * w) & (pix[:, 1] >= 0) & (pix[:, 1] < h)
        pix = pix[inside]
        if len(pix) >= 2:
            cv2.polylines(out, [pix], False, LANE_COLORS[i], 2, cv2.LINE_AA)
        cv2.putText(out, f'L{i}:{prob[i]:.2f}', (8, 20 + 18 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, LANE_COLORS[i], 1, cv2.LINE_AA)
    for i in range(2):
        pts3 = np.stack([X_IDXS, edges[i, :, 0], edges[i, :, 1]], axis=1)
        px, valid = project_points(pts3, cam_K, euler, cam_height)
        pix = px[valid].astype(np.int32)
        inside = (pix[:, 0] >= -w) & (pix[:, 0] < 2 * w) & (pix[:, 1] >= 0) & (pix[:, 1] < h)
        pix = pix[inside]
        if len(pix) >= 2:
            cv2.polylines(out, [pix], False, EDGE_COLOR, 1, cv2.LINE_AA)
    return out


def run_sequence(frame_paths, cam_K, euler, cam_height, out_dir, stride=1):
    os.makedirs(os.path.join(out_dir, 'overlays'), exist_ok=True)
    sess = ort.InferenceSession(MODEL_PATH, providers=['CPUExecutionProvider'])

    M_med = get_warp_matrix(euler, cam_K, bigmodel_frame=False)
    M_big = get_warp_matrix(euler, cam_K, bigmodel_frame=True)

    features = np.zeros((1, 24, 512), dtype=np.float16)
    desire = np.zeros((1, 25, 8), dtype=np.float16)
    traffic = np.array([[1.0, 0.0]], dtype=np.float16)   # 한국: 우측통행(LHD) → is_rhd=False
    action_t = np.array([[0.3, 0.9]], dtype=np.float16)

    prev6_med = prev6_big = None
    results = []
    for k, path in enumerate(frame_paths[::stride]):
        bgr = cv2.imread(path)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        cur6_med = rgb_to_yuv6ch(warp_to_model(rgb, M_med))
        cur6_big = rgb_to_yuv6ch(warp_to_model(rgb, M_big))
        if prev6_med is None or prev6_big is None:
            prev6_med, prev6_big = cur6_med, cur6_big
            continue
        img = np.concatenate([prev6_med, cur6_med])[None].astype(np.uint8)
        big_img = np.concatenate([prev6_big, cur6_big])[None].astype(np.uint8)
        out = sess.run(None, {'img': img, 'big_img': big_img,
                              'features_buffer': features, 'desire_pulse': desire,
                              'traffic_convention': traffic, 'action_t': action_t})
        vec = np.asarray(out[0], dtype=np.float32)[0]
        # features 큐 갱신: 최고(最古) 탈락, 최신 hidden_state 를 말미에 (compile_modeld.py 등가)
        features = np.concatenate([features[:, 1:], vec[SLICES['hidden_state']].astype(np.float16)[None, None]], axis=1)
        prev6_med, prev6_big = cur6_med, cur6_big

        lanes, prob, edges = parse_mdn_lanes(vec)
        name = os.path.basename(path)
        cv2.imwrite(os.path.join(out_dir, 'overlays', name),
                    draw_overlay(bgr, lanes, prob, edges, cam_K, euler, cam_height))
        results.append({'frame': name, 'prob': prob.tolist(),
                        'lanes_y0': lanes[:, 0, 0].tolist()})
        if k % 25 == 0:
            print(f'[{k}/{len(frame_paths[::stride])}] prob={np.round(prob, 2)}')

    with open(os.path.join(out_dir, 'lanes.json'), 'w') as f:
        json.dump(results, f, indent=1)
    return results


def calibrate_sequence(frame_paths, cam_K, euler0, stride=1, speed_min=1.0):
    """openpilot calibrationd 원리의 미니 구현 — 모델 pose(ego-motion) 병진 방향으로
    장착각 추정. 직진 주행 프레임에서 trans=[tx,ty,tz](device: x전방·y우·z하)를 모아
    pitch = atan2(-tz, tx), yaw = atan2(ty, tx) 를 산출한다. 반복 호출로 수렴시킨다."""
    sess = ort.InferenceSession(MODEL_PATH, providers=['CPUExecutionProvider'])
    M_med = get_warp_matrix(euler0, cam_K, False)
    M_big = get_warp_matrix(euler0, cam_K, True)
    features = np.zeros((1, 24, 512), dtype=np.float16)
    desire = np.zeros((1, 25, 8), dtype=np.float16)
    tc = np.array([[1.0, 0.0]], dtype=np.float16)
    at = np.array([[0.3, 0.9]], dtype=np.float16)
    prev = None
    trans_list = []
    for path in frame_paths[::stride]:
        rgb = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2RGB)
        cur = (rgb_to_yuv6ch(warp_to_model(rgb, M_med)), rgb_to_yuv6ch(warp_to_model(rgb, M_big)))
        if prev is None:
            prev = cur
            continue
        img = np.concatenate([prev[0], cur[0]])[None]
        big = np.concatenate([prev[1], cur[1]])[None]
        out = sess.run(None, {'img': img, 'big_img': big, 'features_buffer': features,
                              'desire_pulse': desire, 'traffic_convention': tc, 'action_t': at})
        vec = np.asarray(out[0], np.float32)[0]
        features = np.concatenate([features[:, 1:], vec[SLICES['hidden_state']].astype(np.float16)[None, None]], axis=1)
        prev = cur
        pose = vec[SLICES['pose']]
        trans = pose[:3]  # 평균 병진 (m/s), 뒤 절반은 std
        if np.linalg.norm(trans) > speed_min:
            trans_list.append(trans)
    if not trans_list:
        return None
    t = np.mean(trans_list, axis=0)
    pitch = math.atan2(-t[2], t[0])
    yaw = math.atan2(t[1], t[0])
    return {'n': len(trans_list), 'trans': t.tolist(),
            'pitch_delta': pitch, 'yaw_delta': yaw,
            'pitch_total': float(euler0[1] + pitch), 'yaw_total': float(euler0[2] + yaw)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames-dir', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--fov', type=float, default=DEFAULT_FOV_DEG, help='DSS 카메라 HFOV deg (⚠추정)')
    ap.add_argument('--cam-height', type=float, default=0.0,
                    help='view y 병진 미세조정(기본 0 — 모델 z 가 이미 카메라 기준)')
    ap.add_argument('--pitch', type=float, default=0.0, help='카메라 pitch rad (아래+)')
    ap.add_argument('--stride', type=int, default=1)
    ap.add_argument('--calibrate', action='store_true',
                    help='오버레이 대신 pose 기반 장착각 캘리브레이션 수행')
    args = ap.parse_args()

    paths = sorted(glob.glob(os.path.join(args.frames_dir, '*.png')))
    if len(paths) < 2:
        raise SystemExit(f'frames not found: {args.frames_dir}')
    h, w = cv2.imread(paths[0]).shape[:2]
    fl = (w / 2.0) / math.tan(math.radians(args.fov) / 2.0)
    cam_K = np.array([[fl, 0.0, w / 2.0], [0.0, fl, h / 2.0], [0.0, 0.0, 1.0]])
    print(f'cam {w}x{h} fl={fl:.1f} (HFOV {args.fov}°), frames={len(paths)}')

    euler = np.array([0.0, args.pitch, 0.0])
    if args.calibrate:
        cal = calibrate_sequence(paths, cam_K, euler, args.stride)
        if cal is None:
            raise SystemExit('캘리브레이션 실패: 유효 주행 프레임 없음 (속도 부족)')
        os.makedirs(args.out, exist_ok=True)
        with open(os.path.join(args.out, 'calibration.json'), 'w') as f:
            json.dump(cal, f, indent=1)
        print(f"calib n={cal['n']} trans={np.round(cal['trans'],3)} "
              f"delta(pitch={cal['pitch_delta']:+.4f}, yaw={cal['yaw_delta']:+.4f}) "
              f"→ 총 pitch={cal['pitch_total']:.4f} rad ({math.degrees(cal['pitch_total']):.1f}°), "
              f"yaw={cal['yaw_total']:.4f} rad")
        return
    results = run_sequence(paths, cam_K, euler, args.cam_height, args.out, args.stride)
    probs = np.array([r['prob'] for r in results])
    print(f'done. frames={len(results)}, 평균 차선확률(내측 2개)={probs[:, 1:3].mean(axis=0).round(3)}')


if __name__ == '__main__':
    main()
