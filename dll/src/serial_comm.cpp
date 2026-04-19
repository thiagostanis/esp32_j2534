/**
 * serial_comm.cpp  —  ESP32-J2534 V5
 */
#include "serial_comm.h"

BYTE SerialComm::crc8(const BYTE* d, size_t n) {
    BYTE c=0; while(n--) c^=*d++; return c;
}

SerialComm::SerialComm() {
    ackEvent_ = CreateEvent(nullptr, FALSE, FALSE, nullptr);
}
SerialComm::~SerialComm() { close(); CloseHandle(ackEvent_); }

bool SerialComm::open(const std::string& port, DWORD baud) {
    hPort_ = CreateFileA(port.c_str(), GENERIC_READ|GENERIC_WRITE,
                         0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
    if (hPort_==INVALID_HANDLE_VALUE) return false;
    DCB dcb={}; dcb.DCBlength=sizeof(DCB);
    GetCommState(hPort_,&dcb);
    dcb.BaudRate=baud; dcb.ByteSize=8; dcb.StopBits=ONESTOPBIT;
    dcb.Parity=NOPARITY; dcb.fBinary=TRUE;
    dcb.fOutxCtsFlow=FALSE; dcb.fRtsControl=RTS_CONTROL_DISABLE;
    SetCommState(hPort_,&dcb);
    COMMTIMEOUTS to={}; to.ReadIntervalTimeout=1; to.ReadTotalTimeoutConstant=2;
    SetCommTimeouts(hPort_,&to);
    PurgeComm(hPort_, PURGE_RXCLEAR|PURGE_TXCLEAR);
    running_=true;
    rxThread_=std::thread(&SerialComm::rxThreadFn, this);
    return true;
}

void SerialComm::close() {
    running_=false;
    if (rxThread_.joinable()) rxThread_.join();
    if (hPort_!=INVALID_HANDLE_VALUE) { CloseHandle(hPort_); hPort_=INVALID_HANDLE_VALUE; }
}

bool SerialComm::sendFrame(BYTE cmd, const BYTE* pay, UINT16 len) {
    std::lock_guard<std::mutex> lk(txMutex_);
    std::vector<BYTE> f; f.reserve(6+len);
    f.push_back(FRAME_MAGIC0); f.push_back(FRAME_MAGIC1);
    f.push_back(cmd); f.push_back(len&0xFF); f.push_back(len>>8);
    for (UINT16 i=0;i<len;i++) f.push_back(pay[i]);
    f.push_back(crc8(f.data()+2, 3+len));
    OVERLAPPED ov={}; ov.hEvent=CreateEvent(nullptr,TRUE,FALSE,nullptr);
    DWORD w=0; WriteFile(hPort_,f.data(),(DWORD)f.size(),&w,&ov);
    DWORD r=WaitForSingleObject(ov.hEvent,500);
    CloseHandle(ov.hEvent);
    return r==WAIT_OBJECT_0;
}

void SerialComm::rxThreadFn() {
    std::vector<BYTE> buf; buf.reserve(MAX_ISOTP_LEN+32);
    OVERLAPPED ov={}; ov.hEvent=CreateEvent(nullptr,TRUE,FALSE,nullptr);
    while (running_) {
        BYTE tmp[512]; DWORD got=0;
        ResetEvent(ov.hEvent);
        ReadFile(hPort_,tmp,sizeof(tmp),&got,&ov);
        if (WaitForSingleObject(ov.hEvent,50)==WAIT_OBJECT_0) {
            GetOverlappedResult(hPort_,&ov,&got,FALSE);
            for (DWORD i=0;i<got;i++) buf.push_back(tmp[i]);
        }
        while (buf.size()>=6) {
            if (buf[0]!=FRAME_MAGIC0||buf[1]!=FRAME_MAGIC1) { buf.erase(buf.begin()); continue; }
            UINT16 plen=buf[3]|((UINT16)buf[4]<<8);
            size_t flen=6+plen;
            if (buf.size()<flen) break;
            if (buf[5+plen]!=crc8(buf.data()+2,3+plen)) { buf.erase(buf.begin()); continue; }
            parseFrame(std::vector<BYTE>(buf.begin(),buf.begin()+flen));
            buf.erase(buf.begin(),buf.begin()+flen);
        }
    }
    CloseHandle(ov.hEvent);
}

bool SerialComm::parseFrame(const std::vector<BYTE>& frame) {
    BYTE cmd=frame[2];
    UINT16 len=frame[3]|((UINT16)frame[4]<<8);
    const BYTE* pay=frame.data()+5;

    if (cmd==RSP_ACK) {
        lastAckStatus_=(len>=1)?pay[0]:STATUS_OK;
        lastAckExtra_.assign(pay+1, pay+len);
        SetEvent(ackEvent_);
        if (ackCb_) ackCb_(lastAckStatus_);
        return true;
    }

    if (cmd==RSP_MSG_IND) {
        /* [arbId:4][rxId:4][flags:4][ts:4][len:2][data] */
        if (len<18) return true;
        RxFrame rx={}; rx.is_isotp=true;
        memcpy(&rx.arbId,    pay+0, 4);
        memcpy(&rx.rxId,     pay+4, 4);
        memcpy(&rx.flags,    pay+8, 4);
        memcpy(&rx.timestamp,pay+12,4);
        UINT16 dl=pay[16]|((UINT16)pay[17]<<8);
        rx.dataLen=(dl>MAX_ISOTP_LEN)?MAX_ISOTP_LEN:dl;
        memcpy(rx.data, pay+18, rx.dataLen);
        std::lock_guard<std::mutex> lk(rxMutex_); rxQueue_.push(rx);
        return true;
    }

    if (cmd==RSP_CAN_IND) {
        /* [arbId:4][flags:4][ts:4][dlc:1][data] */
        if (len<13) return true;
        RxFrame rx={}; rx.is_isotp=false;
        memcpy(&rx.arbId,    pay+0, 4);
        memcpy(&rx.flags,    pay+4, 4);
        memcpy(&rx.timestamp,pay+8, 4);
        rx.dataLen=pay[12]; if(rx.dataLen>8) rx.dataLen=8;
        memcpy(rx.data, pay+13, rx.dataLen);
        std::lock_guard<std::mutex> lk(rxMutex_); rxQueue_.push(rx);
        return true;
    }
    return true;
}

bool SerialComm::popRxMsg(RxFrame& out) {
    std::lock_guard<std::mutex> lk(rxMutex_);
    if (rxQueue_.empty()) return false;
    out=rxQueue_.front(); rxQueue_.pop(); return true;
}

BYTE SerialComm::sendAndWaitAck(BYTE cmd, const BYTE* pay, UINT16 len, DWORD tout) {
    std::lock_guard<std::mutex> lk(ackMutex_);
    ResetEvent(ackEvent_); lastAckStatus_=0xFF; lastAckExtra_.clear();
    if (!sendFrame(cmd,pay,len)) return STATUS_ERR_HW;
    if (WaitForSingleObject(ackEvent_,tout)!=WAIT_OBJECT_0) return STATUS_ERR_BUSY;
    return lastAckStatus_;
}
