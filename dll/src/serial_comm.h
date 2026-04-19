#pragma once
/**
 * serial_comm.h  —  ESP32-J2534 V5
 */
#include <windows.h>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>
#include <string>
#include "../protocol.h"

struct RxFrame {
    bool     is_isotp;
    uint32_t arbId;
    uint32_t rxId;
    uint32_t flags;
    uint32_t timestamp;
    uint16_t dataLen;
    uint8_t  data[MAX_ISOTP_LEN];
};

class SerialComm {
public:
    SerialComm();
    ~SerialComm();

    bool open(const std::string& port, DWORD baudrate = 2000000);
    void close();
    bool isOpen() const { return hPort_ != INVALID_HANDLE_VALUE; }

    BYTE sendAndWaitAck(BYTE cmd, const BYTE* payload, UINT16 len,
                        DWORD timeoutMs = 2000);

    /* Para IOCTL_GET_PARAM: retorna os bytes extras do ACK */
    std::vector<BYTE> lastAckExtra() const { return lastAckExtra_; }

    bool popRxMsg(RxFrame& out);

    using AckCb = std::function<void(BYTE)>;
    void setAckCallback(AckCb cb) { ackCb_ = cb; }

private:
    bool sendFrame(BYTE cmd, const BYTE* payload, UINT16 len);
    void rxThreadFn();
    bool parseFrame(const std::vector<BYTE>& frame);
    static BYTE crc8(const BYTE* data, size_t len);

    HANDLE              hPort_         = INVALID_HANDLE_VALUE;
    std::thread         rxThread_;
    std::atomic<bool>   running_       { false };
    std::mutex          txMutex_;

    std::queue<RxFrame> rxQueue_;
    std::mutex          rxMutex_;

    AckCb               ackCb_;
    HANDLE              ackEvent_      = INVALID_HANDLE_VALUE;
    std::mutex          ackMutex_;
    BYTE                lastAckStatus_ = 0xFF;
    std::vector<BYTE>   lastAckExtra_;
};
