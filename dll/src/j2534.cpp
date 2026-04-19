/**
 * j2534.cpp  —  ESP32-J2534 V5  (nível OEM)
 *
 * DLL é interface J2534 pura:
 *  - Toda lógica ISO-TP está no ESP32
 *  - Ioctl mapeia ISO15765_BS, ISO15765_STMIN, ISO15765_FRAME_PAD,
 *    DATA_RATE, LOOPBACK, BS, STMIN para parâmetros do ESP32
 *  - PeriodicMsg registrado no scheduler do ESP32
 *  - PassThruWriteMsgs usa CMD_WRITE_ISOTP para ISO-TP,
 *    CMD_WRITE_CAN para CAN puro
 */

#include "../include/j2534.h"

#undef  ERR_BUFFER_EMPTY
#define ERR_BUFFER_EMPTY     0x10
#undef  ERR_BUFFER_OVERFLOW
#define ERR_BUFFER_OVERFLOW  0x12
#undef  CLEAR_PERIODIC_MSGS
#define CLEAR_PERIODIC_MSGS  0x09
#undef  CLEAR_MSG_FILTERS
#define CLEAR_MSG_FILTERS    0x0A

#include "serial_comm.h"
#include "../protocol.h"

#include <string>
#include <map>
#include <mutex>
#include <fstream>
#include <sstream>
#include <vector>

/* ── Globals ─────────────────────────────────────────────────────── */
static SerialComm  g_serial;
static std::mutex  g_mutex;
static ULONG       g_nextCh   = 1;
static ULONG       g_nextFilt = 1;
static ULONG       g_nextPer  = 1;
static long        g_lastErr  = STATUS_NOERROR;

struct FilterData { ULONG maskId, patId, fcId; };

struct Channel {
    ULONG protocolId, flags, baudRate;
    std::map<ULONG,FilterData> filters;
};
static std::map<ULONG,Channel> g_channels;

/* ── Log ──────────────────────────────────────────────────────────── */
static void logDbg(const std::string& m) {
    std::ofstream f("C:\\esp32_j2534\\j2534_esp32_log.txt", std::ios::app);
    if (f.is_open()) f << "[DLL-V5] " << m << "\n";
}
static std::string hexStr(const BYTE* d, size_t n) {
    std::string r; char b[4];
    for (size_t i=0;i<n;i++){sprintf_s(b,"%02X ",d[i]);r+=b;}
    return r;
}
static void setErr(long c) { g_lastErr=c; }

/* ── sendCmd ─────────────────────────────────────────────────────── */
static long sendCmd(BYTE cmd, const BYTE* pay, UINT16 len, DWORD tout=2000) {
    BYTE st=g_serial.sendAndWaitAck(cmd,pay,len,tout);
    switch(st) {
        case STATUS_OK:           return STATUS_NOERROR;
        case STATUS_ERR_BUSY:     return ERR_FAILED;
        case STATUS_ERR_PARAM:    return ERR_INVALID_MSG;
        case STATUS_ERR_HW:       return ERR_FAILED;
        case STATUS_ERR_TIMEOUT:  return ERR_TIMEOUT;
        case STATUS_ERR_ISOTP:    return ERR_FAILED;
        case STATUS_ERR_WAIT_LIMIT: return ERR_FAILED;
        default:                  return ERR_TIMEOUT;
    }
}

/* ── COM port ─────────────────────────────────────────────────────── */
static std::string readComPort() {
    std::ifstream f("C:\\esp32_j2534\\config.ini");
    if (f.is_open()) {
        std::string l;
        while (std::getline(f,l)) {
            if (l.rfind("COM_PORT=",0)==0) {
                std::string p=l.substr(9);
                while (!p.empty()&&(p.back()=='\r'||p.back()==' ')) p.pop_back();
                if (!p.empty()) { logDbg("COM_PORT="+p); return "\\\\.\\"+p; }
            }
        }
    }
    logDbg("config.ini nao encontrado — COM5");
    return "\\\\.\\COM5";
}

/* ── Helpers de addressing ───────────────────────────────────────── */
static ULONG calcRxId(ULONG txId, ULONG flags,
                       const std::map<ULONG,FilterData>& filters) {
    /* 1. FC ID do filtro = rxId do tester = o que a ECU vai responder */
    for (auto& [fid,f] : filters)
        if (f.fcId != 0) return f.fcId;
    /* 2. Fallback por addressing */
    if (flags & CAN_29BIT_ID)
        return (txId & 0xFFFF0000)
             | ((txId & 0x00FF) << 8)
             | ((txId >> 8) & 0x00FF);
    return txId + 8;
}

/* ── Ioctl: mapeia param J2534 → PARAM_* do ESP32 ───────────────── */
static void applyConfigParam(ULONG j2534_param, ULONG value) {
    ULONG espParam = 0;
    switch (j2534_param) {
        case BS:             espParam = PARAM_BS;              break;
        case STMIN:          espParam = PARAM_STMIN;           break;
        case ISO15765_BS:    espParam = PARAM_BS;              break;
        case ISO15765_STMIN: espParam = PARAM_STMIN;           break;
        /* DATA_RATE já foi setado no Connect */
        default: return;
    }
    BYTE pay[12]; UINT16 plen=0;
    ULONG ioc=IOCTL_SET_PARAM;
    memcpy(pay+plen,&ioc,      4); plen+=4;
    memcpy(pay+plen,&espParam, 4); plen+=4;
    memcpy(pay+plen,&value,    4); plen+=4;
    sendCmd(CMD_IOCTL, pay, plen);
    char b[64]; sprintf_s(b,"SET_CONFIG j2534_param=%lu value=%lu",j2534_param,value);
    logDbg(b);
}

static void applyPadding(ULONG flags) {
    /* ISO15765_FRAME_PAD = 0x0040 em TxFlags → habilita padding com 0x00 */
    BYTE pay[12]; UINT16 plen=0;
    ULONG ioc=IOCTL_SET_PARAM;

    ULONG pEn=PARAM_TX_PADDING_EN,  vEn=(flags&ISO15765_FRAME_PAD)?1UL:0UL;
    ULONG pVal=PARAM_TX_PADDING_VAL, vVal=0x00UL;

    memcpy(pay+plen,&ioc,  4);plen+=4;
    memcpy(pay+plen,&pEn,  4);plen+=4;
    memcpy(pay+plen,&vEn,  4);plen+=4;
    sendCmd(CMD_IOCTL,pay,12);

    if (vEn) {
        plen=0;
        memcpy(pay+plen,&ioc,  4);plen+=4;
        memcpy(pay+plen,&pVal, 4);plen+=4;
        memcpy(pay+plen,&vVal, 4);plen+=4;
        sendCmd(CMD_IOCTL,pay,12);
    }
}

/* ================================================================
   API J2534
   ================================================================ */

long WINAPI PassThruOpen(void* pName, ULONG* pDeviceID) {
    logDbg("=== PassThruOpen V5 ===");
    std::lock_guard<std::mutex> lk(g_mutex);
    if (!pDeviceID) return ERR_NULL_PARAMETER;
    if (g_serial.isOpen()) { *pDeviceID=1; return STATUS_NOERROR; }
    std::string port=readComPort();
    if (!g_serial.open(port)) { logDbg("ERRO: nao abriu "+port); return ERR_DEVICE_NOT_CONNECTED; }
    if (sendCmd(CMD_PING,nullptr,0,2000)!=STATUS_NOERROR) {
        logDbg("ERRO: ESP32 sem PING"); g_serial.close(); return ERR_DEVICE_NOT_CONNECTED;
    }
    logDbg("OK"); *pDeviceID=1; return STATUS_NOERROR;
}

long WINAPI PassThruClose(ULONG DeviceID) {
    std::lock_guard<std::mutex> lk(g_mutex);
    g_channels.clear(); g_serial.close(); logDbg("Close"); return STATUS_NOERROR;
}

long WINAPI PassThruConnect(ULONG DeviceID, ULONG ProtocolID,
                             ULONG Flags, ULONG BaudRate, ULONG* pChannelID) {
    std::lock_guard<std::mutex> lk(g_mutex);
    if (!pChannelID) return ERR_NULL_PARAMETER;
    BYTE pay[5];
    pay[0] = (ProtocolID==ISO15765) ? PROTO_ISO15765 : PROTO_CAN;
    memcpy(pay+1,&BaudRate,4);
    long r=sendCmd(CMD_CONNECT,pay,5);
    if (r!=STATUS_NOERROR) return r;
    ULONG chId=g_nextCh++;
    g_channels[chId]={ProtocolID,Flags,BaudRate};
    char b[80]; sprintf_s(b,"Connect chId=%lu proto=%lu baud=%lu flags=0x%08X",chId,ProtocolID,BaudRate,Flags);
    logDbg(b);
    *pChannelID=chId; return STATUS_NOERROR;
}

long WINAPI PassThruDisconnect(ULONG ChannelID) {
    std::lock_guard<std::mutex> lk(g_mutex);
    sendCmd(CMD_DISCONNECT,nullptr,0); g_channels.erase(ChannelID); return STATUS_NOERROR;
}

/* ── PassThruWriteMsgs ──────────────────────────────────────────── */
long WINAPI PassThruWriteMsgs(ULONG ChannelID, PASSTHRU_MSG* pMsg,
                               ULONG* pNumMsgs, ULONG Timeout) {
    if (!pMsg||!pNumMsgs) return ERR_NULL_PARAMETER;
    if (*pNumMsgs==0) return STATUS_NOERROR;
    std::lock_guard<std::mutex> lk(g_mutex);
    if (!g_channels.count(ChannelID)) return ERR_INVALID_CHANNEL_ID;

    Channel& ch=g_channels[ChannelID];
    bool isIso=(ch.protocolId==ISO15765);
    ULONG sent=0;

    for (ULONG i=0;i<*pNumMsgs;i++) {
        PASSTHRU_MSG& m=pMsg[i];
        ULONG arbId=((ULONG)m.Data[0]<<24)|((ULONG)m.Data[1]<<16)
                   |((ULONG)m.Data[2]<<8)|(ULONG)m.Data[3];
        ULONG payLen=m.DataSize>=4?m.DataSize-4:0;
        const BYTE* pl=&m.Data[4];

        ULONG espFlags=0;
        if (m.TxFlags & CAN_29BIT_ID)       espFlags|=MSG_FLAG_29BIT_ID;
        if (m.TxFlags & ISO15765_FRAME_PAD)  espFlags|=MSG_FLAG_TX_PAD;

        char id[12]; sprintf_s(id,"%08X",arbId);

        if (!isIso) {
            /* CAN puro */
            BYTE pay[9+8]; UINT16 p=0;
            memcpy(pay+p,&arbId,   4);p+=4;
            memcpy(pay+p,&espFlags,4);p+=4;
            pay[p++]=(BYTE)payLen;
            memcpy(pay+p,pl,payLen);p+=(UINT16)payLen;
            if (sendCmd(CMD_WRITE_CAN,pay,p,Timeout)!=STATUS_NOERROR) break;
        } else {
            logDbg("TX UDS -> ID:"+std::string(id)+" | "+hexStr(pl,payLen));

            /* Aplica padding se TxFlag indicar */
            applyPadding(m.TxFlags);

            ULONG rxId=calcRxId(arbId,ch.flags,ch.filters);

            /* CMD_WRITE_ISOTP: [txId:4][rxId:4][flags:4][len:2][data] */
            UINT16 plen=14+(UINT16)payLen;
            std::vector<BYTE> pay(plen);
            UINT16 pos=0;
            memcpy(&pay[pos],&arbId,   4);pos+=4;
            memcpy(&pay[pos],&rxId,    4);pos+=4;
            memcpy(&pay[pos],&espFlags,4);pos+=4;
            UINT16 pl16=(UINT16)payLen;
            memcpy(&pay[pos],&pl16,    2);pos+=2;
            memcpy(&pay[pos],pl,payLen);

            /* Timeout generoso para ISO-TP (flash, downloads) */
            DWORD tout=Timeout>10000?Timeout:10000;
            if (sendCmd(CMD_WRITE_ISOTP,pay.data(),plen,tout)!=STATUS_NOERROR) break;
        }
        sent++;
    }
    *pNumMsgs=sent;
    return (sent>0)?STATUS_NOERROR:ERR_FAILED;
}

/* ── PassThruReadMsgs ───────────────────────────────────────────── */
long WINAPI PassThruReadMsgs(ULONG ChannelID, PASSTHRU_MSG* pMsg,
                              ULONG* pNumMsgs, ULONG Timeout) {
    if (!pMsg||!pNumMsgs||*pNumMsgs==0) return ERR_NULL_PARAMETER;
    memset(pMsg,0,sizeof(PASSTHRU_MSG)*(*pNumMsgs));
    if (!g_channels.count(ChannelID)) return ERR_INVALID_CHANNEL_ID;

    Channel& ch=g_channels[ChannelID];
    bool isIso=(ch.protocolId==ISO15765);
    DWORD dl=GetTickCount()+Timeout;
    ULONG got=0;

    while (got<*pNumMsgs) {
        RxFrame rx;
        if (g_serial.popRxMsg(rx)) {
            /* Filtragem */
            if (!ch.filters.empty()) {
                bool pass=false;
                for (auto& [fid,f]:ch.filters)
                    if ((rx.arbId&f.maskId)==f.patId){pass=true;break;}
                if (!pass) continue;
            }
            PASSTHRU_MSG& out=pMsg[got];
            out.ProtocolID=ch.protocolId;
            out.Timestamp =rx.timestamp;
            out.RxStatus  =(rx.flags&MSG_FLAG_29BIT_ID)?CAN_29BIT_ID:0;
            out.Data[0]=(rx.arbId>>24)&0xFF;
            out.Data[1]=(rx.arbId>>16)&0xFF;
            out.Data[2]=(rx.arbId>> 8)&0xFF;
            out.Data[3]= rx.arbId     &0xFF;
            out.DataSize      =rx.dataLen+4;
            out.ExtraDataIndex=out.DataSize;
            memcpy(&out.Data[4],rx.data,rx.dataLen);

            if (isIso && rx.is_isotp) {
                char b[12]; sprintf_s(b,"%08X",rx.arbId);
                logDbg("RX UDS <- ID:"+std::string(b)
                       +" | "+std::to_string(rx.dataLen)+"B"
                       +" | "+hexStr(rx.data,rx.dataLen));
            }
            got++;
        } else {
            if (Timeout==0||GetTickCount()>=dl) break;
            Sleep(1);
        }
    }
    if (got==0){*pNumMsgs=0;setErr(ERR_BUFFER_EMPTY);return ERR_BUFFER_EMPTY;}
    setErr(STATUS_NOERROR); *pNumMsgs=got; return STATUS_NOERROR;
}

/* ── StartMsgFilter ─────────────────────────────────────────────── */
long WINAPI PassThruStartMsgFilter(ULONG ChannelID, ULONG FilterType,
    PASSTHRU_MSG* pMaskMsg, PASSTHRU_MSG* pPatternMsg,
    PASSTHRU_MSG* pFlowControlMsg, ULONG* pMsgID) {
    std::lock_guard<std::mutex> lk(g_mutex);
    if (!pMaskMsg||!pPatternMsg||!pMsgID) return ERR_NULL_PARAMETER;
    auto gID=[](PASSTHRU_MSG* m)->ULONG{
        return ((ULONG)m->Data[0]<<24)|((ULONG)m->Data[1]<<16)
              |((ULONG)m->Data[2]<< 8)|(ULONG)m->Data[3];
    };
    ULONG mask=gID(pMaskMsg), pat=gID(pPatternMsg);
    ULONG fc=(FilterType==0x03&&pFlowControlMsg)?gID(pFlowControlMsg):0;
    BYTE  ftype=(FilterType==BLOCK_FILTER)?FILTER_BLOCK
               :(FilterType==0x03)?FILTER_FLOWCTRL:FILTER_PASS;

    /* CMD_ADD_FILTER: [type:1][maskId:4][patId:4][fcId:4][fid:1] */
    BYTE pay[14]; UINT16 plen=0;
    pay[plen++]=ftype;
    memcpy(pay+plen,&mask,4);plen+=4;
    memcpy(pay+plen,&pat, 4);plen+=4;
    memcpy(pay+plen,&fc,  4);plen+=4;
    ULONG fid=g_nextFilt; pay[plen++]=(BYTE)(fid&0xFF);
    if (sendCmd(CMD_ADD_FILTER,pay,plen)!=STATUS_NOERROR) return ERR_FAILED;
    g_channels[ChannelID].filters[fid]={mask,pat,fc};
    char b[128]; sprintf_s(b,"Filter fid=%lu type=%u mask=%08X pat=%08X fc=%08X",fid,ftype,mask,pat,fc);
    logDbg(b);
    *pMsgID=g_nextFilt++; return STATUS_NOERROR;
}

long WINAPI PassThruStopMsgFilter(ULONG ChannelID, ULONG MsgID) {
    std::lock_guard<std::mutex> lk(g_mutex);
    BYTE pay[1]={(BYTE)(MsgID&0xFF)};
    sendCmd(CMD_DEL_FILTER,pay,1);
    if (g_channels.count(ChannelID)) g_channels[ChannelID].filters.erase(MsgID);
    return STATUS_NOERROR;
}

/* ── PeriodicMsg — scheduler real no ESP32 ─────────────────────── */
long WINAPI PassThruStartPeriodicMsg(ULONG ChannelID, PASSTHRU_MSG* pMsg,
                                     ULONG* pMsgID, ULONG TimeInterval) {
    std::lock_guard<std::mutex> lk(g_mutex);
    if (!pMsg||!pMsgID) return ERR_NULL_PARAMETER;
    if (!g_channels.count(ChannelID)) return ERR_INVALID_CHANNEL_ID;

    ULONG arbId=((ULONG)pMsg->Data[0]<<24)|((ULONG)pMsg->Data[1]<<16)
               |((ULONG)pMsg->Data[2]<<8)|(ULONG)pMsg->Data[3];
    ULONG payLen=pMsg->DataSize>=4?pMsg->DataSize-4:0;
    if (payLen>8) payLen=8;
    ULONG espFlags=(pMsg->TxFlags&CAN_29BIT_ID)?MSG_FLAG_29BIT_ID:0;
    ULONG msgId=g_nextPer;
    if (msgId>=MAX_PERIODIC_MSGS) return ERR_EXCEEDED_LIMIT;

    /* CMD_ADD_PERIODIC: [msgId:1][txId:4][flags:4][period_ms:4][dlc:1][data] */
    BYTE pay[14+8]; UINT16 plen=0;
    pay[plen++]=(BYTE)msgId;
    memcpy(pay+plen,&arbId,       4);plen+=4;
    memcpy(pay+plen,&espFlags,    4);plen+=4;
    memcpy(pay+plen,&TimeInterval,4);plen+=4;
    pay[plen++]=(BYTE)payLen;
    memcpy(pay+plen,&pMsg->Data[4],payLen);plen+=(UINT16)payLen;
    if (sendCmd(CMD_ADD_PERIODIC,pay,plen)!=STATUS_NOERROR) return ERR_FAILED;

    char b[80]; sprintf_s(b,"Periodic msgId=%lu arbId=%08X period=%lums",msgId,arbId,TimeInterval);
    logDbg(b);
    *pMsgID=g_nextPer++; return STATUS_NOERROR;
}

long WINAPI PassThruStopPeriodicMsg(ULONG ChannelID, ULONG MsgID) {
    std::lock_guard<std::mutex> lk(g_mutex);
    BYTE pay[1]={(BYTE)(MsgID&0xFF)};
    sendCmd(CMD_DEL_PERIODIC,pay,1);
    return STATUS_NOERROR;
}

/* ── Ioctl ──────────────────────────────────────────────────────── */
long WINAPI PassThruIoctl(ULONG ChannelID, ULONG IoctlID,
                          void* pInput, void* pOutput) {

    /* Limpezas diretas */
    if (IoctlID==CLEAR_RX_BUFFER||IoctlID==CLEAR_TX_BUFFER
    ||  IoctlID==CLEAR_MSG_FILTERS||IoctlID==CLEAR_PERIODIC_MSGS) {
        BYTE pay[12]={}; memcpy(pay,&IoctlID,4);
        sendCmd(CMD_IOCTL,pay,12); return STATUS_NOERROR;
    }

    /* SET_CONFIG — aplica cada parâmetro no ESP32 */
    if (IoctlID==SET_CONFIG && pInput) {
        auto* list=reinterpret_cast<SCONFIG_LIST*>(pInput);
        for (ULONG i=0;i<list->NumOfParams;i++)
            applyConfigParam(list->ConfigPtr[i].Parameter,
                             list->ConfigPtr[i].Value);
        return STATUS_NOERROR;
    }

    /* GET_CONFIG — retorna valor local se conhecido */
    if (IoctlID==GET_CONFIG && pInput) {
        auto* list=reinterpret_cast<SCONFIG_LIST*>(pInput);
        for (ULONG i=0;i<list->NumOfParams;i++) {
            if (list->ConfigPtr[i].Parameter==DATA_RATE&&g_channels.count(ChannelID))
                list->ConfigPtr[i].Value=g_channels[ChannelID].baudRate;
        }
        return STATUS_NOERROR;
    }

    return STATUS_NOERROR;
}

/* ── Demais ─────────────────────────────────────────────────────── */
long WINAPI PassThruGetLastError(char* pDesc) {
    if (!pDesc) return ERR_NULL_PARAMETER;
    memset(pDesc,0,80);
    if      (g_lastErr==ERR_BUFFER_EMPTY) strcpy_s(pDesc,80,"PassThruReadMsgs : no message available");
    else if (g_lastErr==ERR_TIMEOUT)      strcpy_s(pDesc,80,"PassThruReadMsgs : Read Timeout");
    else if (g_lastErr==STATUS_NOERROR)   strcpy_s(pDesc,80,"STATUS_NOERROR");
    else                                  strcpy_s(pDesc,80,"ERR_FAILED");
    return STATUS_NOERROR;
}

long WINAPI PassThruReadVersion(ULONG, char* pFW, char* pDll, char* pApi) {
    if (pFW)  strcpy_s(pFW, 80,"05.00.00.001");
    if (pDll) strcpy_s(pDll,80,"05.00.00.001");
    if (pApi) strcpy_s(pApi,80,"04.04");
    return STATUS_NOERROR;
}

long WINAPI PassThruSetProgrammingVoltage(ULONG,ULONG,ULONG){return ERR_NOT_SUPPORTED;}

BOOL APIENTRY DllMain(HMODULE,DWORD reason,LPVOID){
    if (reason==DLL_PROCESS_DETACH) g_serial.close();
    return TRUE;
}
