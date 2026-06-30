# Hörmann SupraMatic → ESPHome (HCP2 / UAP1 emulation)

Control a **Hörmann SupraMatic** garage / door drive from **Home Assistant** with a
cheap **ESP32 + RS485 transceiver**. The ESP plugs onto the drive's bus connector
and emulates a genuine **Hörmann UAP1** universal adapter — no cloud, no BiSecur
gateway, fully local.

This is a cleaned-up, **regression-fixed** fork of the excellent work by
[stephan192](https://github.com/stephan192/hoermann_door),
[mariopenzendorfer](https://github.com/mariopenzendorfer/esphome-hoermann_supramatic_3)
and [avshrs](https://github.com/avshrs/ESP32_Hormann_Supramatic_e3).

---

## ✨ What this fork adds

| | |
|---|---|
| 🐛 **Fix: physical remotes dead after an ESPHome update** | The root cause of the "Home Assistant still works but the radio hand transmitters and the wall button stopped reacting" bug — see below. |
| 📊 **Estimated live position (0–100 %)** | The bus only reports open/closed, so this is a time-based estimate that snaps to the real end stops. |
| 🧠 **Self-learning travel times** | Auto-calibrates the open/close duration from every full cycle (moving average) and persists it to flash — survives reboots and keeps improving. |
| 🎚️ **Position slider / "go to X %"** | Drive the door to any percentage from the Home Assistant cover slider (time based). |
| 🧹 **Clean, fully commented code** | English state strings, documented protocol, no dead PIC16 path. |

---

## 🐛 The bug that was fixed (ESPHome ≥ 2026.2.3)

**Symptom:** after updating ESPHome, the door still obeyed Home Assistant, but the
**original Hörmann radio hand transmitters and the wall push-button stopped working**.

**Root cause.** To speak the Hörmann bus protocol, the firmware has to send a "sync
break" before every frame, which it does by briefly switching the UART to a different
baud rate. It used ESPHome's `UARTComponent::load_settings()` to do that switch.

Since **ESPHome 2026.2.3**, `load_settings()` no longer just reconfigures the UART —
it **deletes and re-installs the entire UART driver** on every call
(`uart_driver_delete()` + `uart_driver_install()`):

```
old (≤ 2025.x):  load_settings()  ->  uart_param_config()                  // lightweight
new (≥ 2026.2.3): load_settings() ->  uart_driver_delete()
                                       uart_driver_install()
                                       uart_set_mode()
                                       uart_param_config()                 // full re-install
```

The firmware calls `load_settings()` **twice for every bus answer**, many times per
second. Re-installing the driver that often **glitches the shared half-duplex line and
destroys the strict response timing**. The Hörmann drive then stops trusting the
emulated UAP1 and drops into a degraded/error state in which it **ignores its radio
remotes and wall button** — while a command injected from Home Assistant can still
occasionally slip through. (The Hörmann protocol explicitly says: *"if the slave
doesn't respond the master will show error 7 and the door will not move anymore."*)

**The fix.** This component is ESP-IDF only, so it now changes the baud rate / word
length with the **lightweight ESP-IDF setters** `uart_set_baudrate()` /
`uart_set_word_length()` instead of `load_settings()`. Those only touch the relevant
hardware registers and leave the driver (and its RX buffer) intact — exactly the
behaviour that used to work, and it's both faster and timing-correct.

> See [`components/uapbridge_esp/uapbridge_esp.cpp`](components/uapbridge_esp/uapbridge_esp.cpp) → `transmit()`.

### Don't want to flash this fork right now? (workarounds)

* Pin ESPHome to **2025.8.x** (the last release before the regression), **or**
* build with the **Arduino** framework instead of esp-idf (different UART backend).

Flashing this fork is the proper fix and keeps you on current ESPHome + esp-idf.

---

## 🔌 Hardware & wiring

* An **ESP32** (classic `esp32dev` / WROOM works fine).
* An **RS485 transceiver** (MAX485, or an auto-direction type like MAX13487).
* The drive's bus connector (RS485 `DATA+` / `DATA-`, 24 V, GND).

| Drive bus | ESP / transceiver |
|-----------|-------------------|
| RS485 A / B (DATA+ / DATA-) | transceiver A / B |
| transceiver RO (RX) | ESP **GPIO16** (`rx_pin`) |
| transceiver DI (TX) | ESP **GPIO17** (`tx_pin`) |
| DE/RE direction (only if not auto-direction) | ESP GPIO, set as `rts_pin` |

> ⚠️ The 24 V from the drive must be stepped down to power the ESP — do **not** feed
> it to a 3.3 V pin. See the original projects' schematics for a reference PCB.

Bus parameters are fixed by Hörmann: **19200 baud, 8 data bits, no parity, 1 stop bit.**

---

## 🚀 Installation

1. Copy this repo next to your ESPHome config (the `components/` folder must sit beside
   `hormann.yaml`, as referenced by `external_components: type: local`).
2. `cp secrets.yaml.example secrets.yaml` and fill in your Wi-Fi / API / OTA secrets.
3. Adjust `hormann.yaml` (entity names, and the travel times — see below).
4. Flash once over USB, then OTA afterwards:
   ```bash
   esphome run hormann.yaml
   ```

---

## ⚙️ Configuration

### Position estimation & self-learning (optional)

The Hörmann bus reports only discrete states (open / closed / moving / venting /
error) — **there is no absolute position in the protocol**. To still get a position
slider, the cover estimates the position from the **travel time**:

```yaml
cover:
  - platform: uapbridge
    name: "SupraMatic E3"
    device_class: garage
    auto_calibration: true   # self-learn & persist the travel times
    open_duration: 15s       # initial seed (rough guess is fine)
    close_duration: 15s      # initial seed
```

* While moving, the position is interpolated and published ~once per second.
* When the drive reports the real **open** / **closed** end stop, the position snaps
  to exactly 100 % / 0 % — so small timing errors never accumulate.
* **`auto_calibration: true`** makes the cover **measure the real travel time on
  every full open↔close cycle**, blend it into a moving average, and **save it to
  flash**. The estimate self-corrects over a few cycles and survives reboots of the
  ESP and the drive. `open_duration` / `close_duration` are then just the seed used
  until the first measurement (default 20 s if omitted). Only *full*, uninterrupted
  end-to-end travels are learned from, so partial/reversed moves never corrupt it.
* With a position estimate, the Home Assistant cover shows a **slider**: set it to
  e.g. 50 % and the door moves and is stopped when the estimate reaches the target
  (best effort, time based). Limitation: if you trigger a *separate* full open/close
  from a physical remote in the same direction **while** a "go to X %" move is still
  running, the bus can't tell the two apart and the door may stop at X % — just press
  the remote again.
* **Omit `auto_calibration` and both durations** for a plain open/closed garage door
  without a slider.

### `uapbridge_esp` options

| Option | Default | Description |
|--------|---------|-------------|
| `uart_id` | — | The `uart:` bus id (required). |
| `rts_pin` | unset | DE/RE direction pin; leave unset for auto-direction transceivers. |
| `auto_correction` | `false` | Try to auto-clear a harmless latched error after an interrupted cycle. |

---

## 🧩 Entities exposed

* **Cover** — open / close / stop / toggle (+ estimated position).
* **Switch** — venting (and optionally the light).
* **Light** — drive courtesy light.
* **Buttons** — vent, impulse.
* **Binary sensors** — relay state, error, pre-warning, "valid status received" (bus health).
* **Text sensor** — door state (`Open` / `Closed` / `Opening` / `Closing` / `Stopped` / `Venting` / `Error`).

---

## 🙏 Credits

Protocol reverse-engineering and the original implementations belong to
**stephan192**, **mariopenzendorfer** and **avshrs**. This fork only cleans the code,
fixes the ESPHome 2026.2.3 UART regression and adds time-based position estimation.

## 📄 License

MIT — see [LICENSE](LICENSE).
