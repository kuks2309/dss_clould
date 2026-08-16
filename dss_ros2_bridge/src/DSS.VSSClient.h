#pragma once
#include <string>
#include <functional>
#include <cstdint>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <nats/nats.h>

class DSSVssClient {
public:
    using OnMessage = std::function<void(const std::string& subject, const std::string& payload)>;
    using OnRawMessage = std::function<void(const std::string& subject, const std::vector<uint8_t>& payload)>;
    using OnReply = std::function<void(natsStatus status, const std::string& payload)>;
    using OnResult = std::function<void(natsStatus status)>;

public:
    // 싱글톤 접근
    static DSSVssClient& singleton();

    // 연결 관리
    natsStatus start(const std::string& ip = "172.25.96.1", uint16_t drivePort = 8886,uint16_t vssPort = 4222);
    void stop();
    void setDriveControl(float throttle, float steer, float brake);
    void get(const std::string& vssName,OnReply onReply,int64_t timeoutMs = 1000);
    void set(const std::string& vssName,const std::string& value,OnResult onResult = nullptr,int64_t timeoutMs = 1000);
    void action(const std::string& vssName,const std::string& value,OnReply onReply,int64_t timeoutMs = 2000);
    // -------- SUBSCRIBE --------
    natsSubscription* subscribe(const std::string& subject,OnMessage onMessage,natsStatus* outStatus = nullptr);
    natsSubscription* subscribe(const std::string& subject, OnRawMessage onMessage, natsStatus* outStatus = nullptr);
    // -------- SUBSCRIBE --------
    natsSubscription* subscribeSensor(const std::string& subject, OnRawMessage onMessage, natsStatus* outStatus = nullptr);
    



    // 구독 해제. 구독 핸들과 함께 그 구독이 소유한 콜백 클로저까지 회수한다 —
    // 구독만 파괴하고 클로저 소유 맵에 항목을 남기면 stop() 이 같은 핸들을 다시 파괴한다.
    static void unsubscribe(natsSubscription*& sub);

private:
    DSSVssClient();
    ~DSSVssClient();

    DSSVssClient(const DSSVssClient&) = delete;
    DSSVssClient& operator=(const DSSVssClient&) = delete;

private:
    void requestAsync(const std::string& subject,const std::string& payload,OnReply onReply,int64_t timeoutMs);
    static void natsMsgThunk(natsConnection* nc,natsSubscription* sub,natsMsg* msg,void* closure);
    static void natsRawMsgThunk(natsConnection* nc, natsSubscription* sub, natsMsg* msg, void* closure);


private:
    natsConnection* conn = nullptr;
    natsOptions* opts = nullptr;
    int udp_socket_ = -1;
    sockaddr_in dss_addr_;
};
