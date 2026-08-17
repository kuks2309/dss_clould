#include "dss_ros2_bridge/msg/dss_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "ament_index_cpp/get_package_prefix.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QProcess>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>

#include <memory>
#include <string>
#include <utility>

// DSS 차량 수동 조작(jog) 창 — enable ON 동안 /dss/control/jog (dss_ros2_bridge/DssControl) 를
// 20 Hz 상시 발행한다. 버튼·키를 누르는 동안만 명령이 실리고 떼면 0 (dead-man). ON 상태에서는
// 놓아도 0 명령 스트림을 유지해 수신측(DSSControlNode) watchdog 이 발행자 생존을 알 수 있게 한다.
// enable OFF 면 발행을 전면 중단해 제어권을 /dss/control(자율 노드)에 넘긴다 — 소스 선택은
// 수신측 mux 가 /dss/jog_enabled (latched Bool) 로 판정한다 (설계 근거: docs/adr/ jog-enable-mux).
// 값 범위(dss.proto 주석): steer -1.0~+1.0, throttle 0.0~1.0, brake 0.0~1.0. 후진은 target_gear=-1.
// steer 부호는 ◀=음수 가정이며 실기 미검증 — 반대로 돌면 buildUi·applyKey 의 ±1 을 뒤집는다.
class JogControlWindow : public QWidget
{
  public:
    explicit JogControlWindow(rclcpp::Node::SharedPtr node) : node_(std::move(node))
    {
        // 명령 토픽은 reliable — 수신측(DSSControlNode)의 QoS 와 일치해야 연결된다.
        pub_ = node_->create_publisher<dss_ros2_bridge::msg::DssControl>("/dss/control/jog",
                                                                         rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
        // enable 상태는 latched(transient_local) — 수신측이 늦게 떠도 현재 상태를 받는다.
        enable_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/dss/jog_enabled", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

        buildUi();
        publishEnabled(); // 기본 OFF 를 즉시 latch — 수신측이 자율 소스로 시작하게 한다
        updateEnableUi();

        auto *timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, [this]() { publishCommand(); });
        timer->start(kPublishPeriodMs);
    }

  protected:
    void keyPressEvent(QKeyEvent *event) override
    {
        if (event->isAutoRepeat() || !applyKey(event->key(), true))
        {
            QWidget::keyPressEvent(event);
        }
    }

    void keyReleaseEvent(QKeyEvent *event) override
    {
        if (event->isAutoRepeat() || !applyKey(event->key(), false))
        {
            QWidget::keyReleaseEvent(event);
        }
    }

    void closeEvent(QCloseEvent *event) override
    {
        if (enable_button_->isChecked())
        {
            enable_button_->setChecked(false); // toggled 시그널 → OFF 발행 (수신측을 자율 소스로 복귀)
        }
        stopBridgeNode(); // 내가 띄운 수신 노드만 정리 — 외부에서 기동된 노드는 건드리지 않는다
        QWidget::closeEvent(event);
    }

  private:
    void buildUi()
    {
        setWindowTitle("DSS Jog Control");

        status_label_ = new QLabel(this);
        status_label_->setAlignment(Qt::AlignCenter);

        bridge_button_ = new QPushButton(this);
        bridge_button_->setFocusPolicy(Qt::NoFocus);
        connect(bridge_button_, &QPushButton::clicked, this, [this]() { startBridgeNode(); });

        // enable 토글 — 문구·조작 위젯 활성화는 updateEnableUi 가 상태에 맞춰 갱신한다.
        enable_button_ = new QPushButton(this);
        enable_button_->setCheckable(true);
        enable_button_->setFocusPolicy(Qt::NoFocus); // Space 가 토글을 건드리면 안 된다 (정지 키와 충돌)
        enable_button_->setMinimumHeight(44);
        enable_button_->setStyleSheet("QPushButton:checked { background-color: #2e7d32; color: white; }");
        connect(enable_button_, &QPushButton::toggled, this, [this](bool on) { onJogToggled(on); });

        auto *btn_forward = makeJogButton(QString::fromUtf8("▲\n전진"));
        auto *btn_left = makeJogButton(QString::fromUtf8("◀ 좌조향"));
        auto *btn_stop = makeJogButton(QString::fromUtf8("■ 정지"));
        auto *btn_right = makeJogButton(QString::fromUtf8("우조향 ▶"));
        auto *btn_reverse = makeJogButton(QString::fromUtf8("▼\n후진"));

        connect(btn_forward, &QPushButton::pressed, this, [this]() { drive_dir_ = 1; });
        connect(btn_forward, &QPushButton::released, this, [this]() { drive_dir_ = 0; });
        connect(btn_left, &QPushButton::pressed, this, [this]() { steer_dir_ = -1; });
        connect(btn_left, &QPushButton::released, this, [this]() { steer_dir_ = 0; });
        connect(btn_right, &QPushButton::pressed, this, [this]() { steer_dir_ = 1; });
        connect(btn_right, &QPushButton::released, this, [this]() { steer_dir_ = 0; });
        connect(btn_reverse, &QPushButton::pressed, this, [this]() { drive_dir_ = -1; });
        connect(btn_reverse, &QPushButton::released, this, [this]() { drive_dir_ = 0; });
        // 정지: 누르는 동안 전 조작 해제 + 풀 브레이크, 떼면 브레이크도 해제(dead-man 일관)
        connect(btn_stop, &QPushButton::pressed, this, [this]() {
            releaseAll();
            braking_ = true;
        });
        connect(btn_stop, &QPushButton::released, this, [this]() { braking_ = false; });

        auto *pad = new QGridLayout();
        pad->addWidget(btn_forward, 0, 1);
        pad->addWidget(btn_left, 1, 0);
        pad->addWidget(btn_stop, 1, 1);
        pad->addWidget(btn_right, 1, 2);
        pad->addWidget(btn_reverse, 2, 1);

        throttle_slider_ = makeScaleSlider(kDefaultThrottlePercent);
        steer_slider_ = makeScaleSlider(kDefaultSteerPercent);
        throttle_value_ = new QLabel(this);
        steer_value_ = new QLabel(this);

        auto *scales = new QGridLayout();
        scales->addWidget(new QLabel(QString::fromUtf8("Throttle 상한"), this), 0, 0);
        scales->addWidget(throttle_slider_, 0, 1);
        scales->addWidget(throttle_value_, 0, 2);
        scales->addWidget(new QLabel(QString::fromUtf8("Steer 강도"), this), 1, 0);
        scales->addWidget(steer_slider_, 1, 1);
        scales->addWidget(steer_value_, 1, 2);

        auto *hint = new QLabel(
            QString::fromUtf8("키보드: ↑/W 전진 · ↓/S 후진 · ←/A →/D 조향 · Space 정지(브레이크) — 조그 ON 일 때만"),
            this);
        hint->setAlignment(Qt::AlignCenter);

        auto *top = new QHBoxLayout();
        top->addWidget(status_label_, 1);
        top->addWidget(bridge_button_);

        // 조작 위젯 묶음 — enable OFF 시 setEnabled(false) 로 일괄 비활성(회색) 처리한다.
        controls_ = new QWidget(this);
        auto *controls_box = new QVBoxLayout(controls_);
        controls_box->setContentsMargins(0, 0, 0, 0);
        controls_box->addLayout(pad);
        controls_box->addLayout(scales);

        auto *root = new QVBoxLayout(this);
        root->addLayout(top);
        root->addWidget(enable_button_);
        root->addWidget(controls_);
        root->addWidget(hint);
    }

    // 키보드 조작을 조작 상태에 반영한다. 처리한 키면 true. 조그 OFF 면 전부 무시한다.
    bool applyKey(int key, bool pressed)
    {
        if (!enable_button_->isChecked())
        {
            return false; // 제어권이 자율 노드에 있는 동안 키 입력이 상태를 바꾸면 안 된다
        }
        switch (key)
        {
        case Qt::Key_Up:
        case Qt::Key_W:
            drive_dir_ = pressed ? 1 : 0;
            return true;
        case Qt::Key_Left:
        case Qt::Key_A:
            steer_dir_ = pressed ? -1 : 0;
            return true;
        case Qt::Key_Right:
        case Qt::Key_D:
            steer_dir_ = pressed ? 1 : 0;
            return true;
        case Qt::Key_Down:
        case Qt::Key_S:
            drive_dir_ = pressed ? -1 : 0;
            return true;
        case Qt::Key_Space:
            if (pressed)
            {
                releaseAll();
            }
            braking_ = pressed;
            return true;
        default:
            return false;
        }
    }

    // 조작 상태 → DssControl 환산 (제동 중 스로틀 0, 후진 시 target_gear=-1).
    dss_ros2_bridge::msg::DssControl makeCommand() const
    {
        dss_ros2_bridge::msg::DssControl msg;
        msg.throttle = (drive_dir_ != 0 && !braking_) ? throttleScale() : 0.0F;
        msg.steer = static_cast<float>(steer_dir_) * steerScale();
        msg.brake = braking_ ? 1.0F : 0.0F;
        msg.target_gear = drive_dir_ < 0 ? -1.0F : 0.0F; // -1=후진 (DSS 실측 확정)
        return msg;
    }

    // 조그 ON 이면 조작 상태를 발행하고, OFF 면 발행 없이 표시만 갱신한다 (QTimer 20 Hz).
    void publishCommand()
    {
        const dss_ros2_bridge::msg::DssControl msg = makeCommand();
        if (enable_button_->isChecked())
        {
            pub_->publish(msg);
            status_label_->setText(QString::fromUtf8("수신 노드 %1 · 기어 %2 · steer %3 · throttle %4 · brake %5")
                                       .arg(pub_->get_subscription_count())
                                       .arg(QString::fromUtf8(drive_dir_ < 0 ? "R" : "D"))
                                       .arg(msg.steer, 0, 'f', 2)
                                       .arg(msg.throttle, 0, 'f', 2)
                                       .arg(msg.brake, 0, 'f', 2));
        }
        else
        {
            status_label_->setText(QString::fromUtf8("조그 OFF — 제어권: /dss/control(자율 노드) · 수신 노드 %1")
                                       .arg(pub_->get_subscription_count()));
        }

        throttle_value_->setText(QString::number(throttleScale(), 'f', 2));
        steer_value_->setText(QString::number(steerScale(), 'f', 2));
        updateBridgeButton();
    }

    // enable 토글 처리 — 조작 상태를 비우고, OFF 전환이면 0 명령 1회 후 침묵한다(안전 벨트).
    void onJogToggled(bool on)
    {
        releaseAll();
        if (!on)
        {
            pub_->publish(dss_ros2_bridge::msg::DssControl()); // 전 필드 0 — 수신측 전환 지연 대비
        }
        publishEnabled();
        updateEnableUi();
    }

    // 현재 enable 상태를 latched 토픽으로 알린다 — 수신측 mux 의 소스 선택 근거.
    void publishEnabled()
    {
        std_msgs::msg::Bool msg;
        msg.data = enable_button_->isChecked();
        enable_pub_->publish(msg);
    }

    // enable 상태를 토글 문구·조작 위젯 활성화에 반영한다.
    void updateEnableUi()
    {
        const bool on = enable_button_->isChecked();
        controls_->setEnabled(on);
        enable_button_->setText(on ? QString::fromUtf8("● 조그 ON — 클릭하여 해제")
                                   : QString::fromUtf8("조그 OFF — 클릭하여 활성화"));
    }

    // 수신측 연결 상태에 따라 버튼을 갱신한다 — 연결되면 비활성, 미연결이면 실행 버튼.
    void updateBridgeButton()
    {
        const bool connected = pub_->get_subscription_count() > 0;
        const bool starting =
            !connected && bridge_process_ != nullptr && bridge_process_->state() != QProcess::NotRunning;
        bridge_button_->setEnabled(!connected && !starting);
        bridge_button_->setText(connected  ? QString::fromUtf8("● 수신 연결됨")
                                : starting ? QString::fromUtf8("수신 노드 시작 중…")
                                           : QString::fromUtf8("수신 노드 실행"));
    }

    // 수신 노드(DSSControlNode)를 자식 프로세스로 기동한다. GUI 종료 시 closeEvent 가 함께 정리한다.
    void startBridgeNode()
    {
        if (bridge_process_ == nullptr)
        {
            bridge_process_ = new QProcess(this);
            bridge_process_->setProcessChannelMode(QProcess::ForwardedChannels); // 노드 로그를 GUI 터미널로
        }
        if (bridge_process_->state() != QProcess::NotRunning)
        {
            return;
        }
        try
        {
            const std::string prefix = ament_index_cpp::get_package_prefix("dss_ros2_bridge");
            bridge_process_->start(QString::fromStdString(prefix + "/lib/dss_ros2_bridge/DSSControlNode"),
                                   QStringList());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "dss_ros2_bridge 패키지를 찾지 못했습니다: %s", e.what());
        }
    }

    void stopBridgeNode()
    {
        if (bridge_process_ != nullptr && bridge_process_->state() != QProcess::NotRunning)
        {
            bridge_process_->terminate();
            if (!bridge_process_->waitForFinished(2000))
            {
                bridge_process_->kill();
            }
        }
    }

    void releaseAll()
    {
        drive_dir_ = 0;
        braking_ = false;
        steer_dir_ = 0;
    }

    float throttleScale() const
    {
        return static_cast<float>(throttle_slider_->value()) / 100.0F;
    }

    float steerScale() const
    {
        return static_cast<float>(steer_slider_->value()) / 100.0F;
    }

    QPushButton *makeJogButton(const QString &text)
    {
        auto *btn = new QPushButton(text, this);
        // 버튼이 포커스를 가지면 Space·화살표를 먹어 키보드 조작이 죽는다 — 포커스는 창이 유지한다.
        btn->setFocusPolicy(Qt::NoFocus);
        btn->setMinimumSize(96, 64);
        return btn;
    }

    QSlider *makeScaleSlider(int initial_percent)
    {
        auto *slider = new QSlider(Qt::Horizontal, this);
        slider->setRange(0, 100);
        slider->setValue(initial_percent);
        slider->setFocusPolicy(Qt::NoFocus);
        return slider;
    }

    static constexpr int kPublishPeriodMs = 50;        // 발행 주기 (20 Hz)
    static constexpr int kDefaultThrottlePercent = 30; // 스로틀 상한 초기값 (0.30)
    static constexpr int kDefaultSteerPercent = 50;    // 조향 강도 초기값 (0.50)

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<dss_ros2_bridge::msg::DssControl>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
    QProcess *bridge_process_ = nullptr; // 버튼으로 띄운 수신 노드 (외부 기동분은 관리하지 않음)
    QPushButton *bridge_button_ = nullptr;
    QPushButton *enable_button_ = nullptr; // checkable — isChecked() 가 조그 enable 의 단일 진실
    QWidget *controls_ = nullptr;          // 조그 패드+슬라이더 묶음 — OFF 시 일괄 비활성
    QLabel *status_label_ = nullptr;
    QLabel *throttle_value_ = nullptr;
    QLabel *steer_value_ = nullptr;
    QSlider *throttle_slider_ = nullptr;
    QSlider *steer_slider_ = nullptr;
    int drive_dir_ = 0; // -1 후진 · 0 정지 · +1 전진
    bool braking_ = false;
    int steer_dir_ = 0; // -1 좌 · 0 중립 · +1 우
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    int exit_code = 0;
    {
        // 창이 노드·퍼블리셔를 보유한다 — DDS 엔티티는 rclcpp::shutdown 전에 소멸해야
        // 종료가 hang 없이 끝난다(소멸 순서 제약). 블록 스코프가 그 순서를 보장한다.
        JogControlWindow window(std::make_shared<rclcpp::Node>("JogControlNode"));
        window.show();
        exit_code = QApplication::exec();
    }
    rclcpp::shutdown();
    return exit_code;
}
