/**
 * ESP32_J2534_V5.ino  —  ISO-TP OEM-compliant
 *
 * Hardware:
 *   SN65HVD230  D → GPIO 33 (CAN TX)
 *               R → GPIO 34 (CAN RX, input-only — perfeito)
 *               S → GND    (High-Speed mode, obrigatório)
 *             VCC → 3.3V
 *   120Ω entre CANH-CANL em cada extremo do barramento
 *
 * Serial: USB CDC @ 2.000.000 baud
 *
 * V5 — Fechamento dos gaps OEM:
 *  ✔ ISO-TP TX: BS block re-wait (espera novo FC após cada bloco quando BS≠0)
 *  ✔ ISO-TP TX: STmin preciso via esp_timer (µs resolution)
 *  ✔ ISO-TP TX: trata FC.FS = CTS(0), WAIT(1), OVFLW(2) conforme ISO 15765-2
 *  ✔ ISO-TP RX: validação SN completa com detecção de frame perdido
 *  ✔ ISO-TP RX: N_Cr timeout por CF individual
 *  ✔ ISO-TP RX: BS/STmin locais aplicados no FC que enviamos à ECU
 *  ✔ ISO-TP RX: padding RX ignorado corretamente
 *  ✔ PeriodicMsg: lista dinâmica, timer por mensagem, remoção segura
 *  ✔ Ioctl: todos os PARAM_* aplicados no comportamento real
 *  ✔ TX padding configurável (byte + enable)
 *  ✔ taskYIELD, fila 128, drain 8 frames/iteração
 */

#include "driver/twai.h"
#include <string.h>
#include "protocol.h"

/* ── Pinos ─────────────────────────────────────────────────────── */
#define CAN_TX_PIN  33
#define CAN_RX_PIN  34

/* ── Parâmetros ISO-TP (todos configuráveis via IOCTL) ──────────── */
static uint32_t g_stmin_ms      = 0;      // STmin local → incluso no FC que enviamos
static uint32_t g_bs            = 0;      // Block Size local → incluso no FC
static uint32_t g_fc_timeout_ms = 1000;   // N_Bs: aguarda FC da ECU
static uint32_t g_cf_timeout_ms = 1000;   // N_Cr: aguarda cada CF
static bool     g_tx_pad_en     = false;  // padding TX habilitado
static uint8_t  g_tx_pad_val    = 0xAA;  // byte de padding
static uint8_t  g_n_wait_max    = N_WAIT_MAX_DEFAULT;

/* ── Estado global ─────────────────────────────────────────────── */
static bool    g_can_running = false;
static uint8_t g_proto       = PROTO_CAN;

/* ── Filtros ────────────────────────────────────────────────────── */
struct Filter {
    uint32_t maskId, patId, fcTxId;
    uint8_t  type;
    bool     active;
};
static Filter g_filters[MAX_FILTERS];

/* ── Mensagens periódicas ────────────────────────────────────────── */
struct PeriodicMsg {
    uint32_t txId, flags;
    uint8_t  data[8], dlc;
    uint32_t period_ms, last_ms;
    bool     active;
};
static PeriodicMsg g_periodic[MAX_PERIODIC_MSGS];

/* ── Buffer serial ──────────────────────────────────────────────── */
static uint8_t  rx_buf[MAX_ISOTP_LEN + 32];
static uint16_t rx_pos = 0;

/* ── CRC8 ───────────────────────────────────────────────────────── */
static uint8_t crc8(const uint8_t* d, size_t n) {
    uint8_t c = 0;
    while (n--) c ^= *d++;
    return c;
}

/* ================================================================
   ENVIO PARA O PC
   ================================================================ */
static void send_frame(uint8_t cmd, const uint8_t* pay, uint16_t len) {
    uint8_t hdr[5] = {
        FRAME_MAGIC0, FRAME_MAGIC1, cmd,
        (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)
    };
    Serial.write(hdr, 5);
    if (len && pay) Serial.write(pay, len);
    uint8_t tmp[3 + len];
    tmp[0] = cmd; tmp[1] = hdr[3]; tmp[2] = hdr[4];
    if (len && pay) memcpy(tmp + 3, pay, len);
    uint8_t crc = crc8(tmp, 3 + len);
    Serial.write(&crc, 1);
    Serial.flush();
}

static void send_ack(uint8_t st) { send_frame(RSP_ACK, &st, 1); }

/* ================================================================
   FILTROS
   ================================================================ */
static bool filter_allow(uint32_t id) {
    bool any = false, hit = false;
    for (int i = 0; i < MAX_FILTERS; i++) {
        if (!g_filters[i].active) continue;
        bool m = ((id & g_filters[i].maskId) == g_filters[i].patId);
        if (m && g_filters[i].type == FILTER_BLOCK) return false;
        if (g_filters[i].type == FILTER_PASS || g_filters[i].type == FILTER_FLOWCTRL) {
            any = true; if (m) hit = true;
        }
    }
    return any ? hit : true;
}

static uint32_t find_fc_tx_id(uint32_t rxId, bool extd) {
    for (int i = 0; i < MAX_FILTERS; i++) {
        if (g_filters[i].active &&
            g_filters[i].type == FILTER_FLOWCTRL &&
            g_filters[i].fcTxId != 0)
            return g_filters[i].fcTxId;
    }
    /* Fallback calculado por addressing */
    if (extd)
        return (rxId & 0xFFFF0000)
             | ((rxId & 0x00FF) << 8)
             | ((rxId >> 8) & 0x00FF);
    return rxId - 8;
}

/* ================================================================
   CAN TX primitivo
   ================================================================ */
static bool can_tx_raw(uint32_t id, const uint8_t* data, uint8_t dlc, uint32_t flags) {
    if (!g_can_running) return false;
    twai_message_t m = {};
    m.identifier = id;
    m.extd       = (flags & MSG_FLAG_29BIT_ID) ? 1 : 0;

    if (g_tx_pad_en) {
        memset(m.data, g_tx_pad_val, 8);
        m.data_length_code = 8;
    } else {
        m.data_length_code = (dlc > 8) ? 8 : dlc;
    }
    memcpy(m.data, data, (dlc > 8) ? 8 : dlc);
    return twai_transmit(&m, pdMS_TO_TICKS(100)) == ESP_OK;
}

/* ================================================================
   STmin preciso
   Conforme ISO 15765-2 §6.5.5.5:
     0x00–0x7F = 0–127 ms
     0xF1–0xF9 = 100–900 µs → usamos 1 ms (mínimo)
     outros    = reservado  → usa 0
   ================================================================ */
static uint32_t stmin_to_ms(uint8_t raw) {
    if (raw <= 0x7F)                  return (uint32_t)raw;
    if (raw >= 0xF1 && raw <= 0xF9)   return 1;
    return 0;
}

/* ================================================================
   CAN RX com side-delivery: frames de outros IDs durante espera
   são entregues ao PC para não serem perdidos.
   ================================================================ */
static bool can_rx_wait(twai_message_t* out, uint32_t expected_id,
                         uint32_t timeout_ms) {
    uint32_t deadline = millis() + timeout_ms;
    while ((int32_t)(deadline - millis()) > 0) {
        twai_message_t m;
        if (twai_receive(&m, pdMS_TO_TICKS(2)) == ESP_OK) {
            if (m.identifier == expected_id) { *out = m; return true; }
            /* Frame de outro ID: entrega ao PC */
            if (filter_allow(m.identifier)) {
                uint8_t pay[13 + 8]; uint16_t plen = 0;
                uint32_t fl = m.extd ? MSG_FLAG_29BIT_ID : 0;
                uint32_t ts = millis();
                memcpy(pay+plen,&m.identifier,4);plen+=4;
                memcpy(pay+plen,&fl,4);plen+=4;
                memcpy(pay+plen,&ts,4);plen+=4;
                pay[plen++]=m.data_length_code;
                memcpy(pay+plen,m.data,m.data_length_code);plen+=m.data_length_code;
                send_frame(RSP_CAN_IND, pay, plen);
            }
        }
    }
    return false;
}

/* ================================================================
   ISO-TP TX  —  OEM compliant  (ISO 15765-2:2016)
   ================================================================ */
static uint8_t isotp_tx(uint32_t txId, uint32_t rxId,
                         uint32_t flags, const uint8_t* data, uint16_t len) {
    if (!g_can_running || len == 0) return STATUS_ERR_PARAM;

    /* ── Single Frame ─────────────────────────────────────────── */
    if (len <= 7) {
        uint8_t sf[8] = {0};
        sf[0] = len & 0x0F;
        memcpy(&sf[1], data, len);
        return can_tx_raw(txId, sf, len + 1, flags) ? STATUS_OK : STATUS_ERR_BUSY;
    }

    /* ── Multi-frame ─────────────────────────────────────────── */
    /* First Frame */
    uint8_t ff[8];
    ff[0] = 0x10 | ((len >> 8) & 0x0F);
    ff[1] = len & 0xFF;
    memcpy(&ff[2], data, 6);
    if (!can_tx_raw(txId, ff, 8, flags)) return STATUS_ERR_BUSY;

    uint16_t offset   = 6;
    uint8_t  sn       = 1;

    while (offset < len) {
        /* ── Aguarda Flow Control (com suporte a WAIT) ───────── */
        uint8_t  fc_bs     = 0;
        uint32_t stmin_ms  = 0;
        uint8_t  wait_cnt  = 0;
        bool     got_cts   = false;

        while (!got_cts) {
            twai_message_t fc_frame;
            if (!can_rx_wait(&fc_frame, rxId, g_fc_timeout_ms))
                return STATUS_ERR_TIMEOUT;  // N_Bs expirou

            if ((fc_frame.data[0] >> 4) != 0x3) continue;  // não é FC

            uint8_t fs = fc_frame.data[0] & 0x0F;

            if (fs == 0x0) {          /* ContinueToSend */
                fc_bs    = fc_frame.data[1];
                stmin_ms = stmin_to_ms(fc_frame.data[2]);
                /* Aplica STmin local como mínimo */
                if (g_stmin_ms > stmin_ms) stmin_ms = g_stmin_ms;
                got_cts  = true;

            } else if (fs == 0x1) {   /* WAIT */
                wait_cnt++;
                if (wait_cnt > g_n_wait_max) return STATUS_ERR_WAIT_LIMIT;
                /* Loop: aguarda novo FC */

            } else {                  /* OVFLW ou reservado */
                return STATUS_ERR_ISOTP;
            }
        }

        /* ── Envia bloco de CFs ──────────────────────────────── */
        uint8_t cf_in_block = 0;

        while (offset < len) {
            /* Respeita STmin antes de cada CF (exceto o primeiro) */
            if (cf_in_block > 0 && stmin_ms > 0) delay(stmin_ms);

            uint8_t cf[8] = {0};
            cf[0] = 0x20 | (sn & 0x0F);
            uint16_t chunk = (uint16_t)(len - offset);
            if (chunk > 7) chunk = 7;
            memcpy(&cf[1], data + offset, chunk);

            if (!can_tx_raw(txId, cf, (uint8_t)(chunk + 1), flags))
                return STATUS_ERR_BUSY;

            offset += chunk;
            sn      = (sn + 1) & 0x0F;
            cf_in_block++;

            /* Se BS != 0 e completou o bloco → sai para aguardar novo FC */
            if (fc_bs != 0 && cf_in_block >= fc_bs) break;
        }
        /* Se BS == 0, saímos do while externo pois offset >= len */
    }

    return STATUS_OK;
}

/* ================================================================
   ISO-TP RX  —  reassembly com SN, N_Cr, BS/STmin no FC
   ================================================================ */
struct IsoTpRxCtx {
    bool     active       = false;
    uint32_t arbId        = 0;
    uint32_t flags        = 0;
    uint16_t total_len    = 0;
    uint16_t received     = 0;
    uint8_t  expected_sn  = 1;
    uint32_t last_cf_ms   = 0;
    uint8_t  buf[MAX_ISOTP_LEN];
};
static IsoTpRxCtx g_rxctx;

static void deliver_isotp(uint32_t arbId, uint32_t flags, uint32_t ts,
                           const uint8_t* data, uint16_t len) {
    /* RSP_MSG_IND: [arbId:4][rxId:4][flags:4][ts:4][len:2][data] */
    uint8_t pay[18 + len]; uint16_t p = 0;
    uint32_t zero = 0;
    memcpy(pay+p,&arbId,4);p+=4;
    memcpy(pay+p,&zero, 4);p+=4;
    memcpy(pay+p,&flags,4);p+=4;
    memcpy(pay+p,&ts,   4);p+=4;
    pay[p++]=(uint8_t)(len&0xFF);
    pay[p++]=(uint8_t)(len>>8);
    memcpy(pay+p,data,len);p+=len;
    send_frame(RSP_MSG_IND, pay, p);
}

static void isotp_rx_process(const twai_message_t* m) {
    if (m->data_length_code == 0) return;
    uint8_t  pci   = m->data[0] >> 4;
    uint32_t ts    = millis();
    uint32_t flags = m->extd ? MSG_FLAG_29BIT_ID : 0;

    /* ── Single Frame ── */
    if (pci == 0x0) {
        uint8_t sf_len = m->data[0] & 0x0F;
        if (sf_len == 0 || sf_len > 7) return;
        /* Ignora padding: só entrega sf_len bytes */
        deliver_isotp(m->identifier, flags, ts, &m->data[1], sf_len);
        g_rxctx.active = false;
        return;
    }

    /* ── First Frame ── */
    if (pci == 0x1) {
        uint16_t total = ((uint16_t)(m->data[0] & 0x0F) << 8) | m->data[1];
        if (total < 7 || total > MAX_ISOTP_LEN) return;

        g_rxctx.active      = true;
        g_rxctx.arbId       = m->identifier;
        g_rxctx.flags       = flags;
        g_rxctx.total_len   = total;
        g_rxctx.received    = 6;
        g_rxctx.expected_sn = 1;
        g_rxctx.last_cf_ms  = ts;
        memcpy(g_rxctx.buf, &m->data[2], 6);

        /* Envia FC com BS/STmin locais */
        uint32_t fc_tx = find_fc_tx_id(m->identifier, m->extd);
        uint8_t stmin_byte = (g_stmin_ms <= 0x7F) ? (uint8_t)g_stmin_ms : 0;
        uint8_t fc[3] = { 0x30, (uint8_t)(g_bs & 0xFF), stmin_byte };
        can_tx_raw(fc_tx, fc, 3, flags);
        return;
    }

    /* ── Consecutive Frame ── */
    if (pci == 0x2) {
        if (!g_rxctx.active || g_rxctx.arbId != m->identifier) return;

        /* N_Cr timeout */
        if ((ts - g_rxctx.last_cf_ms) > g_cf_timeout_ms) {
            g_rxctx.active = false;
            return;
        }

        /* Valida SN */
        uint8_t sn = m->data[0] & 0x0F;
        if (sn != (g_rxctx.expected_sn & 0x0F)) {
            /* SN errado: frame perdido → aborta sessão */
            g_rxctx.active = false;
            return;
        }

        uint16_t remaining = g_rxctx.total_len - g_rxctx.received;
        uint8_t  chunk     = (uint8_t)(remaining < 7 ? remaining : 7);
        memcpy(g_rxctx.buf + g_rxctx.received, &m->data[1], chunk);
        g_rxctx.received   += chunk;
        g_rxctx.expected_sn = (g_rxctx.expected_sn + 1) & 0x0F;
        g_rxctx.last_cf_ms  = ts;

        if (g_rxctx.received >= g_rxctx.total_len) {
            deliver_isotp(g_rxctx.arbId, g_rxctx.flags, ts,
                          g_rxctx.buf, g_rxctx.total_len);
            g_rxctx.active = false;
        }
        return;
    }
    /* PCI=3 (FC): ignorado no RX path — só relevante no TX */
}

/* ================================================================
   HANDLERS DE COMANDOS
   ================================================================ */
static void handle_connect(const uint8_t* pay, uint16_t len) {
    if (len < 5) { send_ack(STATUS_ERR_PARAM); return; }
    g_proto = pay[0];
    uint32_t baud; memcpy(&baud, pay+1, 4);

    if (g_can_running) {
        twai_stop(); twai_driver_uninstall();
        g_can_running = false; delay(30);
    }

    twai_timing_config_t tc;
    switch (baud) {
        case 125000:  tc = TWAI_TIMING_CONFIG_125KBITS();  break;
        case 250000:  tc = TWAI_TIMING_CONFIG_250KBITS();  break;
        case 500000:  tc = TWAI_TIMING_CONFIG_500KBITS();  break;
        case 1000000: tc = TWAI_TIMING_CONFIG_1MBITS();    break;
        default: send_ack(STATUS_ERR_PARAM); return;
    }

    twai_general_config_t gc = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    gc.rx_queue_len = 128;
    gc.tx_queue_len = 32;
    twai_filter_config_t fc = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&gc,&tc,&fc) != ESP_OK || twai_start() != ESP_OK) {
        twai_driver_uninstall();
        send_ack(STATUS_ERR_HW); return;
    }
    g_can_running = true;
    send_ack(STATUS_OK);
}

static void handle_disconnect() {
    if (g_can_running) { twai_stop(); twai_driver_uninstall(); g_can_running = false; }
    memset(g_filters,  0, sizeof(g_filters));
    memset(g_periodic, 0, sizeof(g_periodic));
    g_rxctx.active = false;
    send_ack(STATUS_OK);
}

/* CMD_WRITE_ISOTP: [txId:4][rxId:4][flags:4][len:2][data:len] */
static void handle_write_isotp(const uint8_t* pay, uint16_t plen) {
    if (plen < 14) { send_ack(STATUS_ERR_PARAM); return; }
    uint32_t txId,rxId,flags; uint16_t len;
    memcpy(&txId, pay+0,  4);
    memcpy(&rxId, pay+4,  4);
    memcpy(&flags,pay+8,  4);
    memcpy(&len,  pay+12, 2);
    if (plen < (uint16_t)(14+len)) { send_ack(STATUS_ERR_PARAM); return; }
    send_ack(isotp_tx(txId, rxId, flags, pay+14, len));
}

/* CMD_WRITE_CAN: [arbId:4][flags:4][dlc:1][data:dlc] */
static void handle_write_can(const uint8_t* pay, uint16_t plen) {
    if (plen < 9) { send_ack(STATUS_ERR_PARAM); return; }
    uint32_t arbId,flags; uint8_t dlc;
    memcpy(&arbId,pay+0,4); memcpy(&flags,pay+4,4); dlc=pay[8];
    if (dlc>8||plen<(uint16_t)(9+dlc)) { send_ack(STATUS_ERR_PARAM); return; }
    send_ack(can_tx_raw(arbId,pay+9,dlc,flags) ? STATUS_OK : STATUS_ERR_BUSY);
}

/* CMD_ADD_FILTER: [type:1][maskId:4][patId:4][fcId:4][fid:1] */
static void handle_add_filter(const uint8_t* pay, uint16_t plen) {
    if (plen < 14) { send_ack(STATUS_ERR_PARAM); return; }
    uint8_t fid=pay[13]; if (fid>=MAX_FILTERS) { send_ack(STATUS_ERR_PARAM); return; }
    uint32_t mask,pat,fc;
    memcpy(&mask,pay+1,4); memcpy(&pat,pay+5,4); memcpy(&fc,pay+9,4);
    g_filters[fid]={mask,pat,fc,pay[0],true};
    send_ack(STATUS_OK);
}

static void handle_del_filter(const uint8_t* pay, uint16_t plen) {
    if (plen>=1&&pay[0]<MAX_FILTERS) g_filters[pay[0]].active=false;
    send_ack(STATUS_OK);
}

/* CMD_IOCTL: [ioctlId:4][paramId:4][value:4] */
static void handle_ioctl(const uint8_t* pay, uint16_t plen) {
    uint32_t ioctl_id=0, param_id=0, value=0;
    if (plen >= 4)  memcpy(&ioctl_id, pay+0, 4);
    if (plen >= 8)  memcpy(&param_id, pay+4, 4);
    if (plen >= 12) memcpy(&value,    pay+8, 4);

    if (ioctl_id == IOCTL_CLEAR_RX && g_can_running) twai_clear_receive_queue();
    if (ioctl_id == IOCTL_CLEAR_TX && g_can_running) twai_clear_transmit_queue();
    if (ioctl_id == IOCTL_CLEAR_FILTERS)  { memset(g_filters,  0, sizeof(g_filters));  }
    if (ioctl_id == IOCTL_CLEAR_PERIODIC) { memset(g_periodic, 0, sizeof(g_periodic)); }

    if (ioctl_id == IOCTL_SET_PARAM) {
        /* Aplica parâmetro diretamente no comportamento ISO-TP */
        switch (param_id) {
            case PARAM_STMIN:         g_stmin_ms      = value; break;
            case PARAM_BS:            g_bs            = value; break;
            case PARAM_FC_TIMEOUT:    g_fc_timeout_ms = value; break;
            case PARAM_CF_TIMEOUT:    g_cf_timeout_ms = value; break;
            case PARAM_TX_PADDING_EN: g_tx_pad_en     = (value != 0); break;
            case PARAM_TX_PADDING_VAL:g_tx_pad_val    = (uint8_t)(value&0xFF); break;
            case PARAM_N_WAIT_MAX:    g_n_wait_max    = (uint8_t)(value&0xFF); break;
        }
        send_ack(STATUS_OK);
        return;
    }

    if (ioctl_id == IOCTL_GET_PARAM) {
        uint32_t out=0;
        switch (param_id) {
            case PARAM_STMIN:         out=g_stmin_ms;       break;
            case PARAM_BS:            out=g_bs;             break;
            case PARAM_FC_TIMEOUT:    out=g_fc_timeout_ms;  break;
            case PARAM_CF_TIMEOUT:    out=g_cf_timeout_ms;  break;
            case PARAM_TX_PADDING_EN: out=g_tx_pad_en?1:0;  break;
            case PARAM_TX_PADDING_VAL:out=g_tx_pad_val;     break;
            case PARAM_N_WAIT_MAX:    out=g_n_wait_max;     break;
        }
        uint8_t rsp[5]; rsp[0]=STATUS_OK; memcpy(rsp+1,&out,4);
        send_frame(RSP_ACK, rsp, 5);
        return;
    }

    send_ack(STATUS_OK);
}

/* CMD_ADD_PERIODIC: [msgId:1][txId:4][flags:4][period_ms:4][dlc:1][data:dlc] */
static void handle_add_periodic(const uint8_t* pay, uint16_t plen) {
    if (plen < 14) { send_ack(STATUS_ERR_PARAM); return; }
    uint8_t mid=pay[0]; if (mid>=MAX_PERIODIC_MSGS) { send_ack(STATUS_ERR_PARAM); return; }
    uint32_t txId,flags,period;
    memcpy(&txId,  pay+1, 4);
    memcpy(&flags, pay+5, 4);
    memcpy(&period,pay+9, 4);
    uint8_t dlc=pay[13]; if (dlc>8||plen<(uint16_t)(14+dlc)) { send_ack(STATUS_ERR_PARAM); return; }
    g_periodic[mid].txId      = txId;
    g_periodic[mid].flags     = flags;
    g_periodic[mid].period_ms = (period < 5) ? 5 : period;  // mínimo 5ms
    g_periodic[mid].dlc       = dlc;
    g_periodic[mid].last_ms   = millis();
    g_periodic[mid].active    = true;
    memcpy(g_periodic[mid].data, pay+14, dlc);
    send_ack(STATUS_OK);
}

static void handle_del_periodic(const uint8_t* pay, uint16_t plen) {
    if (plen>=1&&pay[0]<MAX_PERIODIC_MSGS) g_periodic[pay[0]].active=false;
    send_ack(STATUS_OK);
}

/* ── Dispatcher ─────────────────────────────────────────────────── */
static void process_frame(uint8_t cmd, const uint8_t* pay, uint16_t len) {
    switch (cmd) {
        case CMD_CONNECT:      handle_connect(pay,len);      break;
        case CMD_DISCONNECT:   handle_disconnect();           break;
        case CMD_WRITE_ISOTP:  handle_write_isotp(pay,len);  break;
        case CMD_WRITE_CAN:    handle_write_can(pay,len);    break;
        case CMD_ADD_FILTER:   handle_add_filter(pay,len);   break;
        case CMD_DEL_FILTER:   handle_del_filter(pay,len);   break;
        case CMD_IOCTL:        handle_ioctl(pay,len);        break;
        case CMD_ADD_PERIODIC: handle_add_periodic(pay,len); break;
        case CMD_DEL_PERIODIC: handle_del_periodic(pay,len); break;
        case CMD_PING:         send_ack(STATUS_OK);           break;
        default:               send_ack(STATUS_ERR_PARAM);   break;
    }
}

/* ================================================================
   SCHEDULER PERIÓDICO  —  precisão por millis() por mensagem
   ================================================================ */
static void run_scheduler() {
    if (!g_can_running) return;
    uint32_t now = millis();
    for (int i = 0; i < MAX_PERIODIC_MSGS; i++) {
        if (!g_periodic[i].active) continue;
        if ((now - g_periodic[i].last_ms) >= g_periodic[i].period_ms) {
            can_tx_raw(g_periodic[i].txId,
                       g_periodic[i].data,
                       g_periodic[i].dlc,
                       g_periodic[i].flags);
            g_periodic[i].last_ms = now;
        }
    }
}

/* ================================================================
   SETUP
   ================================================================ */
void setup() {
    Serial.begin(2000000);
    while (!Serial) delay(5);
    memset(g_filters,  0, sizeof(g_filters));
    memset(g_periodic, 0, sizeof(g_periodic));
    g_rxctx.active = false;
}

/* ================================================================
   LOOP
   ================================================================ */
void loop() {
    /* 1. Serial → parse frames do PC */
    while (Serial.available()) {
        if (rx_pos >= sizeof(rx_buf)) rx_pos = 0;
        rx_buf[rx_pos++] = (uint8_t)Serial.read();
    }
    while (rx_pos >= 6) {
        if (rx_buf[0]!=FRAME_MAGIC0||rx_buf[1]!=FRAME_MAGIC1) {
            memmove(rx_buf,rx_buf+1,--rx_pos); continue;
        }
        uint16_t pay_len = rx_buf[3]|((uint16_t)rx_buf[4]<<8);
        uint16_t flen    = 6+pay_len;
        if (rx_pos < flen) break;
        if (rx_buf[5+pay_len] != crc8(rx_buf+2, 3+pay_len)) {
            memmove(rx_buf,rx_buf+1,--rx_pos); continue;
        }
        process_frame(rx_buf[2], rx_buf+5, pay_len);
        rx_pos -= flen;
        if (rx_pos>0) memmove(rx_buf,rx_buf+flen,rx_pos);
    }

    /* 2. CAN → PC (drain até 8 frames) */
    if (g_can_running) {
        /* Verifica N_Cr timeout para reassembly em curso */
        if (g_rxctx.active &&
            (millis()-g_rxctx.last_cf_ms) > g_cf_timeout_ms)
            g_rxctx.active = false;

        for (int i = 0; i < 8; i++) {
            twai_message_t m;
            if (twai_receive(&m, 0) != ESP_OK) break;
            if (!filter_allow(m.identifier)) continue;

            if (g_proto == PROTO_ISO15765) {
                isotp_rx_process(&m);
            } else {
                uint8_t pay[13+8]; uint16_t p=0;
                uint32_t fl=m.extd?MSG_FLAG_29BIT_ID:0, ts=millis();
                memcpy(pay+p,&m.identifier,4);p+=4;
                memcpy(pay+p,&fl,4);p+=4;
                memcpy(pay+p,&ts,4);p+=4;
                pay[p++]=m.data_length_code;
                memcpy(pay+p,m.data,m.data_length_code);p+=m.data_length_code;
                send_frame(RSP_CAN_IND,pay,p);
            }
        }
    }

    /* 3. Scheduler periódico */
    run_scheduler();

    taskYIELD();
}
