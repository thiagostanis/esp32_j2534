#pragma once
/**
 * protocol.h  —  ESP32-J2534 V5
 *
 * Frame serial:
 *   [0xAA][0x55][CMD:1][LEN_LO:1][LEN_HI:1][PAYLOAD:LEN][CRC8:1]
 *   CRC8 = XOR(CMD, LEN_LO, LEN_HI, PAYLOAD...)
 */

#define FRAME_MAGIC0   0xAA
#define FRAME_MAGIC1   0x55

/* ── Comandos PC → ESP32 ─────────────────────────────────────────── */
#define CMD_CONNECT          0x01  // [proto:1][baud:4 LE]
#define CMD_DISCONNECT       0x02
#define CMD_WRITE_ISOTP      0x03  // [txId:4][rxId:4][flags:4][len:2][data]
#define CMD_WRITE_CAN        0x04  // [arbId:4][flags:4][dlc:1][data:dlc]
#define CMD_ADD_FILTER       0x05  // [type:1][maskId:4][patId:4][fcId:4][fid:1]
#define CMD_DEL_FILTER       0x06  // [fid:1]
#define CMD_IOCTL            0x07  // [ioctlId:4][paramId:4][value:4]
#define CMD_PING             0x08
#define CMD_ADD_PERIODIC     0x09  // [msgId:1][txId:4][flags:4][period_ms:4][dlc:1][data]
#define CMD_DEL_PERIODIC     0x0A  // [msgId:1]

/* ── Respostas ESP32 → PC ────────────────────────────────────────── */
#define RSP_ACK              0x80  // [status:1]
#define RSP_MSG_IND          0x81  // [arbId:4][rxId:4][flags:4][ts:4][len:2][data]
#define RSP_CAN_IND          0x82  // [arbId:4][flags:4][ts:4][dlc:1][data]
#define RSP_ERROR            0x83  // [code:1]

/* ── Protocolos ──────────────────────────────────────────────────── */
#define PROTO_CAN            0x01
#define PROTO_ISO15765       0x02

/* ── Tipos de filtro ─────────────────────────────────────────────── */
#define FILTER_PASS          0x00
#define FILTER_BLOCK         0x01
#define FILTER_FLOWCTRL      0x02

/* ── Flags de mensagem ───────────────────────────────────────────── */
#define MSG_FLAG_29BIT_ID    (1u << 0)
#define MSG_FLAG_TX_PAD      (1u << 1)

/* ── Status ACK ──────────────────────────────────────────────────── */
#define STATUS_OK            0x00
#define STATUS_ERR_BUSY      0x01
#define STATUS_ERR_PARAM     0x02
#define STATUS_ERR_HW        0x03
#define STATUS_ERR_TIMEOUT   0x04
#define STATUS_ERR_ISOTP     0x05  // FC=OVFLW ou SN errado
#define STATUS_ERR_WAIT_LIMIT 0x06 // WAIT recebido mais de N_WAIT_MAX vezes

/* ── Ioctl IDs ───────────────────────────────────────────────────── */
#define IOCTL_SET_PARAM      0x01
#define IOCTL_GET_PARAM      0x02
#define IOCTL_CLEAR_RX       0x07
#define IOCTL_CLEAR_TX       0x08
#define IOCTL_CLEAR_PERIODIC 0x09
#define IOCTL_CLEAR_FILTERS  0x0A

/* ── Param IDs ───────────────────────────────────────────────────── */
#define PARAM_STMIN          0x01  // STmin local (ms) para FC enviado ao remetente
#define PARAM_BS             0x02  // Block Size local para FC
#define PARAM_FC_TIMEOUT     0x03  // N_Bs: espera FC da ECU (ms, padrão 1000)
#define PARAM_CF_TIMEOUT     0x04  // N_Cr: espera CF (ms, padrão 1000)
#define PARAM_TX_PADDING_EN  0x05  // 0=off, 1=on
#define PARAM_TX_PADDING_VAL 0x06  // byte de padding (padrão 0xAA)
#define PARAM_N_WAIT_MAX     0x07  // máx de WAIT consecutivos (padrão 10)

/* ── Limites ─────────────────────────────────────────────────────── */
#define MAX_FILTERS          16
#define MAX_PERIODIC_MSGS    8
#define MAX_ISOTP_LEN        4096
#define N_WAIT_MAX_DEFAULT   10
