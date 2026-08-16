/*
* VssClient.cpp (fixed)
*/
#include "DSS.VSSClient.h"
#include "dss.pb.h"
#include <thread>
#include <mutex>
#include <unordered_map>
#include <sstream>
#include <iostream>

namespace {
    using FnPtr = DSSVssClient::OnMessage*;
    using FnRawPtr = DSSVssClient::OnRawMessage*;

    std::mutex gSubMx;
    std::unordered_map<natsSubscription*, FnPtr> gClosures;
    std::unordered_map<natsSubscription*, FnRawPtr> gRawClosures;
}
// 생성자는 연결하지 않는다. 연결 주소·포트는 start() 호출자가 정한다 —
// 생성자가 기본 인자로 먼저 연결해 버리면 이후 start(...) 가 이미 연결됨으로 거절된다.
DSSVssClient::DSSVssClient() = default;
DSSVssClient::~DSSVssClient() {
    stop();
}

DSSVssClient& DSSVssClient::singleton() {
    static DSSVssClient s;
    return s;
}

natsStatus DSSVssClient::start(const std::string& ip,uint16_t drivePort, uint16_t vssPort) {
    if (conn != nullptr)
        return NATS_ERR;

    natsStatus s = natsOptions_Create(&opts);
    if (s != NATS_OK) {
        std::cerr << "[VssClient] natsOptions_Create failed: " << natsStatus_GetText(s) << std::endl;
        return s;
    }

    natsOptions_SetMaxReconnect(opts, 60);
    natsOptions_SetReconnectWait(opts, 500);

    std::ostringstream oss;
    oss << "nats://" << ip << ":" << vssPort;
    const std::string url = oss.str();

    s = natsOptions_SetURL(opts, url.c_str());
    if (s == NATS_OK) {
        s = natsConnection_Connect(&conn, opts);
    }

    if (s != NATS_OK) {
        std::cerr << "[VssClient] connect failed (" << url << "): " << natsStatus_GetText(s) << std::endl;
        natsOptions_Destroy(opts);
        opts = nullptr;
        return s;
    }

    udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ < 0) {
        std::cerr << "[VssClient] Failed to create UDP socket\n";
    } else {
        memset(&dss_addr_, 0, sizeof(dss_addr_));
        dss_addr_.sin_family = AF_INET;
        dss_addr_.sin_port   = htons(drivePort);
        inet_pton(AF_INET, ip.c_str(), &dss_addr_.sin_addr);

        std::cout << "[VssClient] UDP Control Ready: "  << ip << ":" << drivePort << std::endl;
    }    

    std::cout << "[VssClient] connected: " << url << std::endl;

    return NATS_OK;
}

void DSSVssClient::stop() {
    // 구독 정리(클로저 포함)
    {
        std::lock_guard<std::mutex> lock(gSubMx);
        for (auto& kv : gClosures) {
            natsSubscription_Destroy(kv.first);
            delete kv.second;
        }
        gClosures.clear();
    }

    {
        std::lock_guard<std::mutex> lock(gSubMx);
        for (auto& kv : gRawClosures) {
            natsSubscription_Destroy(kv.first);
            delete kv.second;
        }
        gRawClosures.clear();
    }

    if (udp_socket_ >= 0) {
        close(udp_socket_);
        udp_socket_ = -1;
    }

    if (conn) {
        natsConnection_Destroy(conn);
        conn = nullptr;
    }
    if (opts) {
        natsOptions_Destroy(opts);
        opts = nullptr;
    }
    nats_Close();
}

// ------------------------------
// 내부 유틸: 비동기 Request/Reply (인자 순서 수정)
// ------------------------------
void DSSVssClient::requestAsync(const std::string& subject, const std::string& payload, OnReply onReply,int64_t timeoutMs)
{
    std::thread([this, subject, payload, onReply, timeoutMs]() {
        if (conn == nullptr) {
            if (onReply) onReply(NATS_CONNECTION_CLOSED, {});
            return;
        }

        natsMsg* reply = nullptr;
        
        natsStatus s = natsConnection_RequestString(
            &reply,             // [1] out
            conn,               // [2]
            subject.c_str(),    // [3]
            payload.c_str(),    // [4]
            timeoutMs           // [5]
        );

        if (s == NATS_OK && reply != nullptr) {
            const char* data = natsMsg_GetData(reply);
            const int   len = natsMsg_GetDataLength(reply);
            std::string out(data, len);
            natsMsg_Destroy(reply);
            if (onReply) onReply(NATS_OK, out);
        }
        else {
            if (onReply) onReply(s, {});
        }
        }).detach();
}


void DSSVssClient::setDriveControl(float throttle, float steer, float brake, float targetGear)
{
    if (udp_socket_ < 0) {
        std::cerr << "[VssClient] UDP socket not initialized!\n";
        return;
    }

    dss::DssSetControl ctrl;
    ctrl.set_identifier("DSS.ROSBridge.VSSClient");
    ctrl.set_timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    ctrl.set_throttle(throttle);
    ctrl.set_brake(brake);
    ctrl.set_steer(steer);
    ctrl.set_targetgear(targetGear); // -1=후진 (DSS 실측 확정) · 0 은 proto3 직렬화에서 생략됨

    std::string buffer;
    ctrl.SerializeToString(&buffer);

    ssize_t sent = sendto(udp_socket_,buffer.data(),buffer.size(),0,(struct sockaddr*)&dss_addr_,sizeof(dss_addr_));
    if (sent < 0) {
        std::cerr << "[VssClient] UDP send failed\n";
    }    
}

// ------------------------------
// GET / SET / ACTION
// ------------------------------
namespace {
    // VSS 요청 JSON 을 고정 버퍼에 조립한다. 인자는 호출자가 주는 임의 길이 문자열이므로
    // 잘림을 검사해서 잘린 JSON 이 전송되는 것을 막는다.
    constexpr size_t kVssJsonCap = 256;

    bool buildVssJson(char (&out)[kVssJsonCap], const char* format, const char* a, const char* b = nullptr)
    {
        const int n = b ? snprintf(out, kVssJsonCap, format, a, b)
                        : snprintf(out, kVssJsonCap, format, a);
        return n >= 0 && static_cast<size_t>(n) < kVssJsonCap;
    }
}

void DSSVssClient::get(const std::string& vssName,OnReply onReply,int64_t timeoutMs)
{
    char json[kVssJsonCap];
    if (!buildVssJson(json, "{ \"VSSName\": \"%s\" }", vssName.c_str())) {
        std::cerr << "[VssClient] get: VSSName 이 요청 버퍼를 넘칩니다: " << vssName << std::endl;
        if (onReply) onReply(NATS_INVALID_ARG, {});
        return;
    }
    requestAsync("VSS.Get", json, std::move(onReply), timeoutMs);
}

void DSSVssClient::set(const std::string& vssName,const std::string& value,OnResult onResult,int64_t timeoutMs)
{
    char json[kVssJsonCap];
    if (!buildVssJson(json, "{ \"VSSName\": \"%s\", \"Value\": \"%s\" }",
                      vssName.c_str(), value.c_str())) {
        std::cerr << "[VssClient] set: VSSName/Value 가 요청 버퍼를 넘칩니다: " << vssName << std::endl;
        if (onResult) onResult(NATS_INVALID_ARG);
        return;
    }

    requestAsync("VSS.Set", json, [onResult](natsStatus st, const std::string&) {
        if (onResult) {
            onResult(st);
        }
    },timeoutMs);
}

void DSSVssClient::action(const std::string& vssName, const std::string& value,OnReply onReply,int64_t timeoutMs)
{
    char json[kVssJsonCap];
    if (!buildVssJson(json, "{ \"VSSName\": \"%s\", \"Value\": \"%s\" }",
                      vssName.c_str(), value.c_str())) {
        std::cerr << "[VssClient] action: VSSName/Value 가 요청 버퍼를 넘칩니다: " << vssName << std::endl;
        if (onReply) onReply(NATS_INVALID_ARG, {});
        return;
    }
    requestAsync("VSS.Act", json, std::move(onReply), timeoutMs);
}

// ------------------------------
// NATS 콜백 → 람다 bridge (불필요한 캐스팅 제거)
// ------------------------------
void DSSVssClient::natsMsgThunk(natsConnection* /*nc*/,
    natsSubscription* /*sub*/,
    natsMsg* msg,
    void* closure)
{
    auto* cb = reinterpret_cast<OnMessage*>(closure); // 우리가 new 한 std::function*
    if (cb) {
        const char* subj = natsMsg_GetSubject(msg);
        const char* data = natsMsg_GetData(msg);
        const int   len = natsMsg_GetDataLength(msg);

        std::string subject = subj ? subj : "";
        std::string payload(data, len);

        (*cb)(subject, payload);
    }
    natsMsg_Destroy(msg);
}


void DSSVssClient::natsRawMsgThunk(natsConnection* /*nc*/,
    natsSubscription* /*sub*/,
    natsMsg* msg,
    void* closure)
{
    if (closure) {
        // closure 는 subscribe() 가 new 한 OnRawMessage*. 다른 시그니처의 std::function 으로
        // 캐스팅해 호출하면 정의되지 않은 동작이므로 별칭 그대로 복원한다.
        auto* cb = reinterpret_cast<OnRawMessage*>(closure);
        if (cb) {
            const char* subj = natsMsg_GetSubject(msg);
            const char* data = natsMsg_GetData(msg);
            const int len = natsMsg_GetDataLength(msg);

            std::string subject = subj ? subj : "";
            std::vector<uint8_t> payload(data, data + len);

            // 안전한 콜백 호출
            (*cb)(subject, payload);
        }
    }

    // 메시지 메모리 해제
    natsMsg_Destroy(msg);
}

// ------------------------------
// Subscribe
// ------------------------------
natsSubscription* DSSVssClient::subscribe(const std::string& subject,OnMessage onMessage,natsStatus* outStatus)
{
    if (outStatus) *outStatus = NATS_OK;
    if (conn == nullptr) {
        if (outStatus) *outStatus = NATS_CONNECTION_CLOSED;
        return nullptr;
    }
    auto* closure = new OnMessage(std::move(onMessage));
    natsSubscription* sub = nullptr;
    natsStatus s = natsConnection_Subscribe(
        &sub,
        conn,
        subject.c_str(),
        &DSSVssClient::natsMsgThunk,
        closure
    );

    if (s != NATS_OK) {
        delete closure;
        if (outStatus) *outStatus = s;
        return nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(gSubMx);
        gClosures[sub] = closure;
    }

    return sub;
}



// ------------------------------
// Subscribe
// ------------------------------
natsSubscription* DSSVssClient::subscribe(const std::string& subject, OnRawMessage onMessage, natsStatus* outStatus)
{
    if (outStatus) *outStatus = NATS_OK;
    if (conn == nullptr) {
        if (outStatus) *outStatus = NATS_CONNECTION_CLOSED;
        return nullptr;
    }
    auto* closure = new OnRawMessage(std::move(onMessage));
    natsSubscription* sub = nullptr;
    natsStatus s = natsConnection_Subscribe(
        &sub,
        conn,
        subject.c_str(),
        &DSSVssClient::natsRawMsgThunk,
        closure
    );

    if (s != NATS_OK) {
        delete closure;
        if (outStatus) *outStatus = s;
        return nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(gSubMx);
        gRawClosures[sub] = closure;
    }

    return sub;
}



// ------------------------------
// Subscribe (센서용 별칭)
// ------------------------------
natsSubscription* DSSVssClient::subscribeSensor(const std::string& subject, OnRawMessage onMessage, natsStatus* outStatus)
{
    return subscribe(subject, std::move(onMessage), outStatus);
}

// ------------------------------
// Unsubscribe
// ------------------------------
void DSSVssClient::unsubscribe(natsSubscription*& sub)
{
    if (!sub) return;

    {
        std::lock_guard<std::mutex> lock(gSubMx);
        auto it = gClosures.find(sub);
        if (it != gClosures.end()) {
            delete it->second;
            gClosures.erase(it);
        }
        auto rit = gRawClosures.find(sub);
        if (rit != gRawClosures.end()) {
            delete rit->second;
            gRawClosures.erase(rit);
        }
    }

    natsSubscription_Destroy(sub);
    sub = nullptr;
}


