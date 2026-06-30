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

### Method A — from the Home Assistant ESPHome dashboard (recommended, no file copying)

The components are pulled **straight from GitHub**, so you only paste one YAML file.

1. Open **ESPHome Device Builder** in Home Assistant → **New device** (or edit your
   existing one) → **Edit**.
2. Select everything and paste the complete configuration below. It already references
   this repo via `external_components: type: git`.
3. Add your secrets (Wi-Fi / API key / OTA password) in the ESPHome **Secrets** editor —
   the names must match those used below.
4. Set the cover travel times (a rough guess is fine, `auto_calibration` refines them).
5. **Save** → device menu (⋮) → **Install** → **Wirelessly** (USB only needed the very
   first time on a brand-new board).

```yaml
substitutions:
  name: "supramatic-e3"
  friendly_name: "SupraMatic E3"
  open_duration: "15s"     # initial seed; auto_calibration refines it
  close_duration: "15s"

esphome:
  name: hormann
  friendly_name: "${friendly_name}"

esp32:
  board: esp32dev
  framework:
    type: esp-idf

# Components are fetched from this GitHub repo. Pin a release tag (recommended) or use
# ref: main for the latest. A new tag forces ESPHome to re-fetch past its cache.
external_components:
  - source:
      type: git
      url: https://github.com/Nicochr2000/esphome-hoermann-supramatic
      ref: v1.4.0
    components: [uapbridge, uapbridge_esp]
    refresh: 1d

logger:
  level: WARN
  baud_rate: 0          # free the serial UART; logs still go over the network

api:
  encryption:
    key: !secret garagedoor_api_encryption_key
  reboot_timeout: 0s    # keep the door working even if Home Assistant is down

ota:
  - platform: esphome
    password: !secret garagedoor_ota_password

safe_mode:

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  power_save_mode: none
  reboot_timeout: 0s
  ap:
    ssid: "Hormann Fallback Hotspot"
    password: !secret garagedoor_ap_password

captive_portal:

uart:
  id: uart_bus
  baud_rate: 19200
  rx_pin: 16
  tx_pin: 17

uapbridge_esp:
  id: garage_door_comp
  uart_id: uart_bus
  # rts_pin: 4          # only if your transceiver has a DE/RE direction pin
  auto_correction: false

cover:
  - platform: uapbridge
    name: "${friendly_name}"
    device_class: garage
    auto_calibration: true
    open_duration: ${open_duration}
    close_duration: ${close_duration}

switch:
  - platform: uapbridge
    venting_switch:
      id: venting_switch
      name: "${friendly_name} Venting"

binary_sensor:
  - platform: uapbridge
    relay_state:
      name: "Relay state"
    error_state:
      name: "Error"
    prewarn_state:
      name: "Pre-warning"
    got_valid_broadcast:
      name: "Valid status received"

output:
  - platform: uapbridge
    id: gd_light

light:
  - platform: uapbridge
    id: my_light
    name: "${friendly_name} Light"
    output: gd_light

text_sensor:
  - platform: uapbridge
    id: garage_door_state
    name: "State"

button:
  - platform: uapbridge
    vent_button:
      id: button_vent
      name: "${friendly_name} Vent"
    impulse_button:
      id: button_impulse
      name: "${friendly_name} Impulse"
```

### Method B — local components (CLI / offline builds)

Prefer vendoring the component (no dependency on GitHub at build time)? Use
[`hormann.yaml`](hormann.yaml) from this repo (it references `external_components:
type: local`, path `components`), copy the `components/` folder next to it, then:

```bash
cp secrets.yaml.example secrets.yaml   # fill in your secrets
esphome run hormann.yaml               # USB the first time, OTA afterwards
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
