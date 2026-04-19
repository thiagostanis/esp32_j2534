# ESP32-J2534 V5  —  Nível OEM

## Fechamento dos gaps (V4 → V5)

| Gap | V4 | V5 |
|-----|----|----|
| ISO-TP TX: BS block re-wait | ❌ BS ignorado | ✅ Para após cada bloco BS e aguarda novo FC |
| ISO-TP TX: FC.FS WAIT | Parcial | ✅ Loop com contador `g_n_wait_max` (padrão 10) |
| ISO-TP TX: FC.FS OVFLW | Retornava erro genérico | ✅ `STATUS_ERR_ISOTP` propagado à DLL |
| ISO-TP TX: STmin por frame | `delay(stmin_ms)` após o 1º | ✅ `delay` antes de cada CF exceto o 1º no bloco |
| ISO-TP RX: padding ignorado | Entregava bytes extras | ✅ SF entrega exatamente `sf_len` bytes |
| ISO-TP RX: SN rollover | Truncava em 0xF | ✅ `(sn+1) & 0x0F` correto, detecta perda |
| PeriodicMsg: mínimo period | Sem limite | ✅ Mínimo 5ms (evita flood no barramento) |
| Ioctl ISO15765_FRAME_PAD | Ignorado | ✅ `applyPadding()` envia PARAM_TX_PADDING_EN/VAL |
| Ioctl ISO15765_BS/STMIN | Só armazenava | ✅ Repassa ao ESP32 via `IOCTL_SET_PARAM` |
| PARAM_N_WAIT_MAX | Não existia | ✅ Configurável via `IOCTL_SET_PARAM` |
| Timeout WRITE_ISOTP | 5s fixo | ✅ `max(Timeout, 10s)` — suporta flash lento |

---

## Fluxo ISO-TP TX completo (V5)

```
DLL → CMD_WRITE_ISOTP (payload UDS puro)
            ↓
ESP32:   len ≤ 7 → Single Frame
         len > 7 → First Frame
                      ↓
                   loop: aguarda FC (N_Bs = 1000ms)
                      FC.FS = WAIT (0x31) → reespera (até N_WAIT_MAX)
                      FC.FS = OVFLW(0x32) → STATUS_ERR_ISOTP
                      FC.FS = CTS  (0x30) → extrai BS, STmin
                      ↓
                   loop por bloco (BS frames):
                      delay(STmin) antes de cada CF (exceto o 1º)
                      envia CF com SN correto
                      se BS != 0 e bloco completo → volta para aguardar FC
            ↓
ESP32 → RSP_ACK STATUS_OK
```

## Fluxo ISO-TP RX completo (V5)

```
ECU → Single Frame (PCI=0x0):
      entrega exatamente sf_len bytes (padding ignorado)

ECU → First Frame (PCI=0x1):
      inicia reassembly
      envia FC com BS=g_bs, STmin=g_stmin_ms locais
      N_Cr timeout = g_cf_timeout_ms por CF

ECU → Consecutive Frame (PCI=0x2):
      valida SN: (sn != expected_sn & 0x0F) → aborta sessão
      atualiza last_cf_ms (N_Cr reset)
      quando received >= total_len → entrega RSP_MSG_IND
```

---

## Hardware

| Sinal   | GPIO | SN65HVD230 |
|---------|------|------------|
| CAN TX  | **33** | D (pino 1) |
| CAN RX  | **34** | R (pino 4) |
| S (mode)| —    | **GND** (obrigatório) |
| VCC     | 3.3V | VCC |
| GND     | GND  | GND |

120Ω CANH-CANL em cada extremo.

---

## Instalação

1. Grave `firmware/ESP32_J2534_V5.ino` no ESP32
2. Edite `config.ini`: `COM_PORT=COMx`
3. Copie `config.ini` para `C:\esp32_j2534\`
4. Compile a DLL (Win32):
   ```bat
   cd dll && mkdir build && cd build
   cmake .. -G "Visual Studio 17 2022" -A Win32
   cmake --build . --config Release
   mkdir C:\esp32_j2534\bin
   copy Release\esp32_j2534.dll C:\esp32_j2534\bin\
   ```
5. Registre: duplo-clique em `register_j2534.reg` → Sim
6. Dianalyzer → Hardware Options → `ESP32 J2534 Bridge` → 500 kbps

---

## Parâmetros configuráveis (Ioctl)

| Param J2534 | Param ESP32 | Padrão | Descrição |
|-------------|-------------|--------|-----------|
| `ISO15765_BS` / `BS` | `PARAM_BS` | 0 | Block Size (0 = sem limite) |
| `ISO15765_STMIN` / `STMIN` | `PARAM_STMIN` | 0 | STmin local em ms |
| `ISO15765_FRAME_PAD` (TxFlag) | `PARAM_TX_PADDING_EN/VAL` | off | Padding TX com 0x00 |
| — | `PARAM_FC_TIMEOUT` | 1000ms | N_Bs: aguarda FC da ECU |
| — | `PARAM_CF_TIMEOUT` | 1000ms | N_Cr: aguarda cada CF |
| — | `PARAM_N_WAIT_MAX` | 10 | Máx de FC.FS=WAIT consecutivos |

---

## Log

`C:\esp32_j2534\j2534_esp32_log.txt`

- `TX UDS -> ID:... | hex` — requisição enviada
- `RX UDS <- ID:... | NB | hex` — resposta recebida
- `Filter fid=N type=T mask=... pat=... fc=...` — filtro
- `Periodic msgId=N arbId=... period=Nms` — keep-alive
- `SET_CONFIG param=N value=N` — Ioctl aplicado
