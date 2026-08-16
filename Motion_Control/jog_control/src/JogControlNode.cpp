#include "dss_ros2_bridge/msg/dss_control.hpp"
#include "rclcpp/rclcpp.hpp"

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

// DSS 차량 수동 조작(jog) 창 — /dss/control (dss_ros2_bridge/DssControl) 20 Hz 상시 발행.
// 버튼·키를 누르는 동안만 명령이 실리고 떼면 0 (dead-man). 놓은 상태에서도 0 명령 스트림을
// 유지해 수신측(DSSControlNode) watchdog 이 발행자 생존을 알 수 있게 한다.
// 값 범위(dss.proto 주석): steer -1.0~+1.0, throttle 0.0~1.0, brake 0.0~1.0. 후진은 target_gear=-1.
// steer 부호는 ◀=음수 가정이며 실기 미검증 — 반대로 돌면 buildUi·applyKey 의 ±1 을 뒤집는다.
class JogControlWindow : public QWidget
{
  public:
    explicit JogControlWindow(rclcpp::Node::SharedPtr node) : node_(std::move(node))
    {
        // 명령 토픽은 reliable — 수신측(DSSControlNode)의 QoS 와 일치해야 연결된다.
        pub_ = node_->create_publisher<dss_ros2_bridge::msg::DssControl>("/dss/control",
                                                                         rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        buildUi();

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

        auto *hint =
            new QLabel(QString::fromUtf8("키보드: ↑/W 전진 · ↓/S 후진 · ←/A →/D 조향 · Space 정지(브레이크)"), this);
        hint->setAlignment(Qt::AlignCenter);

        auto *top = new QHBoxLayout();
        top->addWidget(status_label_, 1);
        top->addWidget(bridge_button_);

        auto *root = new QVBoxLayout(this);
        root->addLayout(top);
        root->addLayout(pad);
        root->addLayout(scales);
        root->addWidget(hint);
    }

    // 키보드 조작을 조작 상태에 반영한다. 처리한 키면 true.
    bool applyKey(int key, bool pressed)
    {
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

    // 조작 상태를 DssControl 로 환산해 발행하고 상태줄을 갱신한다 (QTimer 20 Hz).
    void publishCommand()
    {
        dss_ros2_bridge::msg::DssControl msg;
        msg.throttle = (drive_dir_ != 0 && !braking_) ? throttleScale() : 0.0F;
        msg.steer = static_cast<float>(steer_dir_) * steerScale();
        msg.brake = braking_ ? 1.0F : 0.0F;
        msg.target_gear = drive_dir_ < 0 ? -1.0F : 0.0F; // -1=후진 (DSS 실측 확정)
        pub_->publish(msg);

        throttle_value_->setText(QString::number(throttleScale(), 'f', 2));
        steer_value_->setText(QString::number(steerScale(), 'f', 2));
        status_label_->setText(QString::fromUtf8("수신 노드 %1 · 기어 %2 · steer %3 · throttle %4 · brake %5")
                                   .arg(pub_->get_subscription_count())
                                   .arg(QString::fromUtf8(drive_dir_ < 0 ? "R" : "D"))
                                   .arg(msg.steer, 0, 'f', 2)
                                   .arg(msg.throttle, 0, 'f', 2)
                                   .arg(msg.brake, 0, 'f', 2));
        updateBridgeButton();
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
    QProcess *bridge_process_ = nullptr; // 버튼으로 띄운 수신 노드 (외부 기동분은 관리하지 않음)
    QPushButton *bridge_button_ = nullptr;
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
    JogControlWindow window(std::make_shared<rclcpp::Node>("JogControlNode"));
    window.show();
    const int exit_code = QApplication::exec();
    rclcpp::shutdown();
    return exit_code;
}
