#pragma once
/**
 * j2534.h – API SAE J2534-1 (04/2004)
 *
 * CORRIGIDO v3 (Padrão ACTIA/PSA):
 * - Valores de Erro ajustados (ERR_BUFFER_EMPTY = 0x10)
 * - Valores de Ioctl ajustados (CLEAR_MSG_FILTERS = 0x0A)
 * - Funções Periódicas incluídas.
 */

#include <windows.h>

/* ProtocolID */
#define CAN          0x05
#define ISO15765     0x06

/* ConnectFlags */
#define CAN_29BIT_ID 0x00000100

/* Filtros */
#define PASS_FILTER  0x00000001
#define BLOCK_FILTER 0x00000002

/* IoctlID */
#define GET_CONFIG                   0x01
#define SET_CONFIG                   0x02
#define READ_VBATT                   0x03
#define CLEAR_TX_BUFFER              0x07
#define CLEAR_RX_BUFFER              0x08
#define CLEAR_PERIODIC_MSGS          0x09 // <- Faltava esse!
#define CLEAR_MSG_FILTERS            0x0A // <- Corrigido de 0x09 para 0x0A!

/* RxStatus flags */
#define TX_MSG_TYPE                  0x0001
#define START_OF_MESSAGE             0x0002
#define ISO15765_FIRST_FRAME         0x0004
#define ISO15765_PADDING_IS_IN_DATA  0x0010

/* TxFlags */
#define ISO15765_FRAME_PAD      0x0040

/* ParamID para SCONFIG */
#define DATA_RATE               0x01
#define LOOPBACK                0x03
#define BS                      0x0A
#define STMIN                   0x0B
#define ISO15765_BS             0x24
#define ISO15765_STMIN          0x25
#define CAN_MIXED_FORMAT        0x8000

/* ------------------------------------------------------------------ */
/* PASSTHRU_MSG — estrutura oficial SAE J2534 com todos os campos      */
/* ------------------------------------------------------------------ */
typedef struct {
    unsigned long ProtocolID;       /* Protocolo do canal (CAN, ISO15765...) */
    unsigned long RxStatus;         /* Flags de status na recepcao           */
    unsigned long TxFlags;          /* Flags para transmissao                */
    unsigned long Timestamp;        /* Timestamp em microsegundos            */
    unsigned long DataSize;         /* Numero de bytes validos em Data[]     */
    unsigned long ExtraDataIndex;   /* Indice onde comecam dados extras      */
    unsigned char Data[4128];       /* Payload (4096 bytes + 32 ISO-TP pad)  */
} PASSTHRU_MSG;

typedef struct {
    ULONG Parameter;
    ULONG Value;
} SCONFIG;

typedef struct {
    ULONG NumOfParams;
    SCONFIG *ConfigPtr;
} SCONFIG_LIST;

/* Codigos de retorno J2534 - CORRIGIDOS PADRÃO ACTIA */
#define STATUS_NOERROR            0x00
#define ERR_NOT_SUPPORTED         0x01
#define ERR_INVALID_CHANNEL_ID    0x02
#define ERR_INVALID_PROTOCOL_ID   0x03
#define ERR_NULL_PARAMETER        0x04
#define ERR_INVALID_IOCTL_VALUE   0x05
#define ERR_INVALID_FLAGS         0x06
#define ERR_FAILED                0x07
#define ERR_DEVICE_NOT_CONNECTED  0x08
#define ERR_TIMEOUT               0x09
#define ERR_INVALID_MSG           0x0A
#define ERR_INVALID_TIME_INTERVAL 0x0B
#define ERR_EXCEEDED_LIMIT        0x0C
#define ERR_INVALID_MSG_ID        0x0D
#define ERR_DEVICE_IN_USE         0x0E
#define ERR_INVALID_IOCTL_ID      0x0F
#define ERR_BUFFER_EMPTY          0x10 // <- CORRIGIDO (era 0x12)
#define ERR_BUFFER_FULL           0x11
#define ERR_BUFFER_OVERFLOW       0x12 // <- CORRIGIDO (era 0x14)
#define ERR_PIN_INVALID           0x13
#define ERR_CHANNEL_IN_USE        0x14
#define ERR_MSG_PROTOCOL_ID       0x15
#define ERR_INVALID_FILTER_ID     0x16 // Atualizado pela tabela da ACTIA
#define ERR_NO_FLOW_CONTROL       0x17
#define ERR_NOT_UNIQUE            0x18
#define ERR_INVALID_BAUDRATE      0x19
#define ERR_INVALID_DEVICE_ID     0x1A

/* ------------------------------------------------------------------ */
/* Exported functions                                                */
/* ------------------------------------------------------------------ */
#ifdef __cplusplus
extern "C" {
#endif

long WINAPI PassThruOpen       (void* pName, ULONG* pDeviceID);
long WINAPI PassThruClose      (ULONG DeviceID);
long WINAPI PassThruConnect    (ULONG DeviceID, ULONG ProtocolID,
                                ULONG Flags, ULONG BaudRate, ULONG* pChannelID);
long WINAPI PassThruDisconnect (ULONG ChannelID);
long WINAPI PassThruReadMsgs   (ULONG ChannelID, PASSTHRU_MSG* pMsg,
                                ULONG* pNumMsgs, ULONG Timeout);
long WINAPI PassThruWriteMsgs  (ULONG ChannelID, PASSTHRU_MSG* pMsg,
                                ULONG* pNumMsgs, ULONG Timeout);

// AS DUAS FUNÇÕES QUE FALTAVAM:
long WINAPI PassThruStartPeriodicMsg(ULONG ChannelID, PASSTHRU_MSG* pMsg, 
                                     ULONG* pMsgID, ULONG TimeInterval);
long WINAPI PassThruStopPeriodicMsg (ULONG ChannelID, ULONG MsgID);

long WINAPI PassThruStartMsgFilter(ULONG ChannelID, ULONG FilterType,
                                   PASSTHRU_MSG* pMaskMsg,
                                   PASSTHRU_MSG* pPatternMsg,
                                   PASSTHRU_MSG* pFlowControlMsg,
                                   ULONG* pMsgID);
long WINAPI PassThruStopMsgFilter (ULONG ChannelID, ULONG MsgID);
long WINAPI PassThruSetProgrammingVoltage(ULONG DeviceID, ULONG PinNumber,
                                          ULONG Voltage);
long WINAPI PassThruReadVersion(ULONG DeviceID,
                                char* pFirmwareVersion,
                                char* pDllVersion,
                                char* pApiVersion);
long WINAPI PassThruGetLastError(char* pErrorDescription);
long WINAPI PassThruIoctl      (ULONG ChannelID, ULONG IoctlID,
                                void* pInput, void* pOutput);

#ifdef __cplusplus
}
#endif