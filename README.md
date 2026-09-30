# VESC CYD Dash

An e-bike dashboard that talks to a VESC over **CAN bus** (via a TJA1050 transceiver) or
**UART** (straight to its COMM port) **and** simultaneously acts as a **VESC Tool BLE
bridge**, so your phone still connects to the controller with the display in the loop.

> **Use at your own risk.** This is hobby firmware for a vehicle you ride, written with
> substantial help from AI. It comes with no warranty. Read the
> [disclaimer](#disclaimer-use-at-your-own-risk) before you put it on a bike.

It started on the **ESP32-2432S028** ("Cheap Yellow Display") and now also builds for two
Waveshare ESP32-C6 touch boards and a Waveshare ESP32-S3 AMOLED board:

| Board | Build environment | Display | Touch |
|---|---|---|---|
| ESP32-2432S028**R** (by far the most common CYD) | `cyd_resistive` | ILI9341 240x320 (TFT_eSPI) | XPT2046, resistive, SPI |
| Capacitive members of the same Sunton family | `cyd_capacitive` (default) | ILI9341 240x320 (TFT_eSPI) | CST816/CST820 or GT911, I2C |
| Waveshare ESP32-C6-LCD-1.9 (touch variant) | `c6_lcd19` | ST7789V2 170x320 (Arduino_GFX) | CST8xx, I2C |
| Waveshare ESP32-C6-Touch-LCD-1.47 | `c6_lcd147` | JD9853 172x320 (Arduino_GFX) | AXS5106L, I2C |
| Waveshare ESP32-S3-Touch-AMOLED-1.64 | `s3_amoled164` | CO5300 AMOLED 280x456 (Arduino_GFX) | FT3168, I2C |

Each environment also has a `_uart` twin (`cyd_resistive_uart`, `c6_lcd19_uart`, ...) that
talks to the VESC over UART instead of CAN - see [Wiring](#wiring).

The CYD layout (landscape):

```
+--------------------------------------------------------------+
| [####battery####] 62%     54.2V              *BLE*   o OK     |
+--------------------------------------+-----------------------+
|                                      |  TRIP      12.4        |
|      4 8 .5   km/h                   |  WH USED   146         |
|                                      |  WH/KM     11.8        |
|   [ regen |======drive====== ] 1240W |  MOTOR C   47          |
+--------------------------------------+-----------------------+
| ( SPORT )  HOLD                     ODO 431.2 km              |
+--------------------------------------------------------------+
```

The C6 and AMOLED boards use the same zones (battery strip, speed, power meter, profile
band) but one full-width telemetry zone instead of a side column - see
[Using it](#using-it). The AMOLED's layout is the C6 one scaled up about 1.5x.

---

## Before you build: which CYD touch panel do you have?

*(CYD boards only - the C6 and AMOLED boards each have exactly one touch controller.)*

The 2.8" CYD ships in two variants and they use completely different touch controllers:

| Board marking | Panel | Controller | Build environment |
|---|---|---|---|
| `ESP32-2432S028R` (by far the most common) | resistive | XPT2046 on SPI | `cyd_resistive` |
| Capacitive members of the same Sunton family | capacitive | CST816/CST820 (or GT911) on I2C | `cyd_capacitive` |

`cyd_capacitive` is the default environment - but if you bought a board labelled
**2432S028R**, it is resistive and you want `cyd_resistive`. Flashing the wrong one just
means touch does nothing; the display still works, so it is a cheap thing to test.

The capacitive driver auto-detects CST816/CST820 at `0x15` and GT911 at `0x5D`/`0x14`
and logs which one answered. If none answers, check `TOUCH_SDA_PIN` / `TOUCH_SCL_PIN`
in `include/config.h` against your board's silkscreen.

---

## Wiring

The dash talks to the VESC over one of two links, chosen at build time:

| | CAN (default) | UART |
|---|---|---|
| Build environment | `cyd_resistive`, `c6_lcd19`, ... | the same name + `_uart`: `cyd_resistive_uart`, `c6_lcd19_uart`, ... |
| Extra hardware | TJA1050 transceiver | none - straight to the VESC's COMM port |
| Telemetry | CAN status broadcasts, plus a one-frame poll for the fault code | polled at 10 Hz |
| PAS | yes | no - see [UART](#uart) |
| Profile `app_mode` switching | any mode | throttle on/off only |
| Dash's own config reads/writes while VESC Tool is connected | yes | held off until it disconnects |

Use CAN if your VESC has a CAN port. UART is for VESCs without one, or when you'd rather
not add a transceiver. The switch is `VESC_LINK_UART` in `config.h`; the `_uart`
environments set it for you.

### Pins

Each board has one pair of pins for the VESC link, whichever link you build. TX is the
board's output: it goes to the TJA1050's `TXD` (CAN) or the VESC's `RX` (UART). RX takes
the TJA1050's `RXD` or the VESC's `TX`. They are `VESC_LINK_TX_PIN` / `VESC_LINK_RX_PIN`
in `config.h`.

| Board | TX | RX |
|---|---|---|
| CYD (both variants) | GPIO22 | GPIO35 |
| `c6_lcd19` (ESP32-C6-LCD-1.9) | GPIO21 | GPIO22 |
| `c6_lcd147` (ESP32-C6-Touch-LCD-1.47) | GPIO7 | GPIO3 |
| `s3_amoled164` (ESP32-S3-Touch-AMOLED-1.64) | GPIO17 | GPIO18 |

**CYD.** It only exposes a handful of GPIOs, on two JST connectors:

* **P3**: `GND`, `GPIO35` (input only), `GPIO22`, `GPIO21` (= LCD backlight, do not use)
* **CN1**: `GND`, `GPIO22`, `GPIO27`, `3V3` (on some batches the GPIO27 pad is not connected)

GPIO35 is input-only, which makes it the only sensible RX pin on the board.

**ESP32-C6-LCD-1.9.** GPIO21/22 are free per Waveshare's schematic. The header also breaks
out `EXIO0-7` right next to them - those are pins on an onboard TCA9554 I2C expander, not
real GPIOs, and can't reach the CAN or UART peripherals. Easy to wire by mistake; check
the schematic, not just the silkscreen, if the link won't come up.

**ESP32-C6-Touch-LCD-1.47.** Unlike the 1.9, this board's touch reset (GPIO20) is really
wired, and most of what looks spare is an ESP32-C6 strapping pin. GPIO7 (documented free)
and GPIO3 (the TF-card MISO, unused by this project) are the two clean choices - so the
TF-card slot can't be used alongside the VESC link.

**ESP32-S3-Touch-AMOLED-1.64.** Header P1 breaks out GPIO1/2/3/5/6/7/8/15/16/17/18, none of
them used on the board, and GPIO17/18 sit side by side at its end. (GPIO3 is an S3
strapping pin, as are 0, 45 and 46, so it's best left alone.) P1 has no ground - use the
`GND` pin on header P2.

### CAN

The ESP32 family's built-in CAN controller (TWAI) does the protocol work - the only extra
hardware you need is a **TJA1050 transceiver** module between the board and the bus:
board TX -> TJA1050 `TXD`, board RX <- TJA1050 `RXD`, TJA1050 `VCC` -> 5V, `GND` -> GND.

Then wire the TJA1050's `CANH`/`CANL` to the VESC's CAN port (twisted pair if the run is
long). The bus needs **120 Ω termination at each end** - most single-VESC setups are
already terminated inside the VESC, so add the second 120 Ω resistor across `CANH`/
`CANL` at the dash end.

### UART

The UART link uses the same [pins](#pins), wired straight to the VESC's COMM port: board
TX -> VESC `RX`, board RX <- VESC `TX`, plus `GND` to the VESC's `GND`. Both sides are
3.3 V logic, so there is no transceiver, level shifter or termination. If nothing ever
answers, swap RX and TX first - it is the most common mistake.

UART gives up two things compared with CAN, both forced by the VESC firmware:

* **No PAS.** The VESC's COMM-port UART only runs while *App to Use* is `UART`,
  `PPM and UART` or `ADC and UART`, and the firmware's PAS inputs sit on that same port's
  RX/TX pins by default. A UART build **won't compile** if any profile uses
  `APP_MODE_PAS` or `APP_MODE_ADC_PAS` - see [Rider profiles](#rider-profiles).
* **Profiles only turn the throttle on or off.** Every mode a profile can name would
  stop the UART the dash is talking over if written as-is, so a UART build writes the app
  that gives the same input with the UART still running: `APP_MODE_ADC` (throttle on)
  becomes `ADC and UART`, and `APP_MODE_NONE` (throttle off) becomes `UART`. In
  `ADC and UART` the ADC app can't put its cruise/reverse buttons on the COMM port's
  RX/TX pins (they are the UART); it reads one button on the servo/PPM pin instead.

A UART also has no addresses, so the dash can't tell a config reply it asked for from one
the phone asked for - which might even have been relayed from another VESC on the CAN bus.
So while VESC Tool is connected through the dash, a UART build holds off its own config
traffic: the battery/geometry fetch and profile `app_mode` changes wait until it
disconnects, and the 10 s save hold is refused (save from VESC Tool instead). Profile
limits still apply straight away, and telemetry keeps flowing, at 2 Hz while the phone is
connected. See
[Over UART](#over-uart) for how the two streams are kept apart otherwise.

### Power

The CYD draws roughly 150-250 mA with the backlight up. Most VESC 5V rails handle that, but
check yours before trusting it - a browning-out ESP32 in traffic is no fun.

### On the VESC side

In VESC Tool:

* **CAN:** under *App Settings → General*, enable **CAN status message 1/2/3/4/5** at a
  rate of at least 20-50 Hz. This is easy to miss and, if skipped, looks exactly like a wiring fault:
  the dash will show erpm/current/duty but temps, Ah, Wh, tachometer and voltage will all
  read stuck at zero.
* In the motor configuration's setup info, set your **motor pole count, gear ratio and
  wheel diameter** (`si_motor_poles`, `si_gear_ratio`, `si_wheel_diameter`) and your pack's
  **battery type, series cell count and capacity** (`si_battery_*`). The dash reads all of
  them from the controller - see [Speed and distance](#speed-and-distance) and
  [Battery gauge](#battery-gauge). The firmware defaults are for an electric skateboard
  (14 poles, 3:1, 83 mm wheel), so an unset VESC gives confidently wrong speeds.
* **CAN:** note the VESC's **CAN ID** and put it in `VESC_CAN_TARGET_ID` in `config.h`.
* **UART:** under *App Settings → General*, set **App to Use** to `UART`, `PPM and UART`
  or `ADC and UART` - nothing else runs the COMM-port UART. *Permanent UART* does not
  help: it is for the VESC's built-in NRF/BLE port, not the COMM port. This only matters
  until the dash connects: from then on each profile sets it to `ADC and UART` or
  `UART`, so a PPM input doesn't survive the first profile the dash applies. Under *App Settings
  → UART*, the baud rate must match `VESC_UART_BAUD` (115200, the firmware default).

**Firmware version.** Several things read fields out of the VESC's serialized configs at
fixed byte offsets: the rider profile's `app_mode` (app config), and speed/distance and the
battery gauge (motor config). Those offsets were derived against bldc **`release_7_00`
(FW 7.0.0)**. A reply of the wrong length is detected and skipped rather than guessed at -
app-mode switching, speed/distance, the battery gauge and the flash write each degrade on
their own, see below - but on any other firmware, re-verify against that version's
`confgenerator.c` (the constants are in `include/vesc_protocol.h`).

---

## Building

1. Install [VS Code](https://code.visualstudio.com/) + the PlatformIO extension.
   (`.vscode/extensions.json` also recommends the pioarduino IDE extension and an ESP
   exception decoder.)
2. Open this folder.
3. Pick the environment (`cyd_capacitive`, `cyd_resistive`, `c6_lcd19` for the
   Waveshare ESP32-C6-LCD-1.9, `c6_lcd147` for the ESP32-C6-Touch-LCD-1.47, or
   `s3_amoled164` for the ESP32-S3-Touch-AMOLED-1.64) in the PlatformIO toolbar - with
   `_uart` on the end for the UART link.
4. Upload.

TFT_eSPI is configured entirely from `platformio.ini` build flags, so you never have to
edit `User_Setup.h` inside the library - an upgrade will not silently wipe your setup.

`c6_lcd19`, `c6_lcd147` and `s3_amoled164` pull their toolchain from a different
PlatformIO platform source (the `pioarduino` fork, needed for ESP32-C6 support and shared
by the S3 board) than the CYD environments do, and use Arduino_GFX (`GFX Library for
Arduino`) instead of TFT_eSPI, which does not reliably support the C6 and has no CO5300
driver for the AMOLED. They also use NimBLE-Arduino v2 where the CYD environments use
v1 - `src/ble_bridge.cpp` compiles against either. Building one of them right after a
CYD environment - in the same `pio run -e a -e b` command, or as the next separate
`pio run` - can fail with a `FRAMEWORK_DIR`/`NoneType` error (`TypeError: ... not
'NoneType'`). That's a package collision between the two platform sources (the log shows
`tool-esptoolpy` being reinstalled on the switch), not a project bug: run the same build
again and it goes through.

The two C6 boards share their CAN transport and UI renderer (`src/vesc_can_idf5.cpp`,
`src/ui_c6.cpp`) but differ in display driver (ST7789V2 vs JD9853) and touch controller
(CST8xx family vs AXS5106L) - handled with `BOARD_C6_LCD147` branches in `ui_c6.cpp`
and `touch_input.cpp` and per-board blocks in `config.h`, rather than separate files,
since most of the logic is identical between the two. The
display's rotation and GRAM column/row offsets (`SCREEN_ROTATION`, `LCD_COL_OFFSET`,
`LCD_ROW_OFFSET`) come from Waveshare's own examples, and `config.h` still marks them as
unverified on real hardware - if the image comes up rotated, mirrored or offset, that is
where to look.

The AMOLED board shares that renderer and CAN transport too (`BOARD_S3_AMOLED164`
branches), with three differences worth knowing:

* **It draws through a framebuffer.** The CO5300 only accepts updates that start on an
  even pixel and are an even number of pixels wide and tall, and it can't rotate in
  hardware. So the dash draws into a 255 KB framebuffer in PSRAM that handles rotation
  itself, and pushes the whole frame whenever something changed. That's why the
  environment uses the 16 MB flash / 8 MB PSRAM board definition. If PSRAM isn't
  available the serial log says so, and it falls back to drawing straight to the panel.
* **Its USB-C port is the S3's own USB**, with no USB-UART chip, so the environment turns
  on USB CDC at boot for the serial console. Brightness is a display command rather than
  a backlight pin.
* **It's an OLED, so static elements can burn in.** The profile band, battery strip and
  power-meter frame sit in the same place for the whole ride. `BACKLIGHT_PCT` sets the
  panel's brightness here; lowering it from the default 90 slows burn-in.

Its pins, 20-column offset and startup sequence come from Waveshare's own example and are
equally unverified on real hardware.

---

## Configure it for your bike

Everything lives in **`include/config.h`**. The values that matter most:

```c
#define VESC_CAN_TARGET_ID  119      // YOUR VESC's CAN controller id
#define USE_IMPERIAL        false    // true -> mph / miles
#define PACK_INTERNAL_R_OHM 0.045f   // whole-pack resistance, for the SoC sag compensation
```

The checked-in `VESC_CAN_TARGET_ID` is the author's controller, not the firmware default
(`0`), so change it. UART builds don't use it - they talk to whichever VESC is on the
other end of the wire. There is no motor pole count, gear ratio, wheel diameter, cell count
or capacity to set here - those come from the VESC, see
[Speed and distance](#speed-and-distance) and [Battery gauge](#battery-gauge).

Other things you may want to touch:

| Setting | What it does |
|---|---|
| `VESC_LINK_UART` | `0` = CAN, `1` = UART. The `_uart` environments set it; changing the default here switches every other environment |
| `VESC_LINK_TX_PIN`, `VESC_LINK_RX_PIN` | the board's two pins for the VESC link, CAN or UART (per board - see [Pins](#pins)) |
| `VESC_UART_BAUD` | UART baud rate - must match the VESC's |
| `VESC_CAN_LOCAL_ID` (100), `VESC_CAN_BLE_ID` (101) | the dash's own addresses on the bus - one for its telemetry poll, a separate one for phone traffic so the two request streams can't collide. Must differ from each other and from any real VESC id |
| `VESC_CAN_BITRATE` | must match the VESC's CAN baud rate |
| `UI_LAYOUT_PORTRAIT` | C6 and AMOLED boards only - portrait (default) or landscape UI |
| `AUTO_BRIGHTNESS` | CYD only - backlight follows the onboard LDR. See [Troubleshooting](#troubleshooting). The C6 and AMOLED boards have no light sensor and use the fixed `BACKLIGHT_PCT` |
| `POWER_BAR_MAX_W`, `POWER_BAR_REGEN_W` | full-scale ends of the power meter |
| `TEMP_WARN_FET_C`, `TEMP_WARN_MOTOR_C` | temperature tiles turn red above these |
| `BLE_ENABLED`, `ENABLE_BLE_SECURITY`, `BLE_SECURITY_PASSKEY` | see [BLE](#how-the-dash-and-the-ble-bridge-share-one-can-bus) |

---

## Speed and distance

Motor pole count, gear ratio and wheel diameter are **read from the VESC** - not
hardcoded. The dash sends `COMM_GET_MCCONF` once (retrying every 2 s until both these and
the [battery fields](#battery-gauge) come back usable, then leaving it alone) and takes
`si_motor_poles`, `si_gear_ratio` and `si_wheel_diameter` from the reply.

These are the same values the VESC uses for its own speed and distance, and the dash uses
the same formulas (`mc_interface_get_speed()` / `_get_distance()` in the firmware), so it
agrees with VESC Tool by construction. If the speed is wrong, it is wrong in VESC Tool
too - fix it there.

Until the geometry arrives, speed shows **`--`** and no trip or odometer distance is
counted: tachometer counts can't be turned into metres without it. At boot that is a
second or two, before anyone is moving. It stays that way if:

* the VESC hasn't replied yet (or the link is down);
* the reply is the wrong length for the firmware this was written against (see
  [Firmware version](#on-the-vesc-side));
* a value is outside a sane range: an odd or zero pole count (magnets come in pairs, so an
  odd count means a misaligned read), a gear ratio outside 0.05-100, or a wheel diameter
  outside 0.01-3 m.

Once the geometry and battery fields have both arrived the dash stops asking, so a change
you make in VESC Tool afterwards isn't picked up until the dash restarts.

---

## Battery gauge

Series cell count, chemistry and pack capacity are **read from the VESC** - not
hardcoded. They come from the same `COMM_GET_MCCONF` reply as the
[geometry](#speed-and-distance): `si_battery_type`, `si_battery_cells` and
`si_battery_ah`. Set those for your pack in VESC Tool.

There is deliberately **no config.h fallback**: a wrong guess baked into firmware is worse
than no reading. Until the VESC has answered - and any time the answer can't be trusted -
the gauge shows **`?`** for the battery percentage and voltage, and range shows `--`.
That happens when:

* the VESC hasn't replied yet (or the link is down);
* the reply is the wrong length for the firmware this was written against (see
  [Firmware version](#on-the-vesc-side));
* the cell count or capacity is outside a sane range (1-200 cells, 0.05-500 Ah);
* the battery type has no state-of-charge curve here. Li-ion (3.0-4.2 V/cell) and LiFePO4
  (2.6-3.6 V/cell) are supported; **lead-acid is not**.

State of charge is estimated per cell from pack voltage, compensated for sag with
`PACK_INTERNAL_R_OHM` so the gauge does not collapse under throttle, and heavily filtered
so it does not twitch. Range is the pack's energy (Ah x cells x nominal cell voltage)
times SoC, divided by your running Wh/km.

---

## Using it

| Gesture | Result |
|---|---|
| **Hold the profile badge for 1.4 s, then let go** | cycles to the next rider profile |
| **Keep holding the badge for 10 s** | writes the *current* profile into the VESC's flash |
| Hold the speed area for 1.4 s | resets the trip |
| Tap the telemetry area | CYD: flips between the two stat pages. C6 and AMOLED: cycles through the telemetry fields, then back to speed |

The **profile badge** is the bottom-left badge on the CYD, the bottom-left half of the
bottom row on the C6 and AMOLED boards in landscape, and the whole bottom band in
portrait.

While the badge is held, a sliver fills along its bottom edge. It fills once towards
the 1.4 s cycle - at which point a toast previews which profile you'd land on
(`RELEASE: SPORT`) - then restarts and fills a second time towards the 10 s write.
**The profile cycle commits on release, not on the threshold**, so a hold that runs all
the way to 10 s never cycles: it saves the profile you were already riding. Let go early
to switch, keep holding to save. See
[Writing a profile to the VESC](#writing-a-profile-to-the-vesc).

**CYD stat pages** (the right-hand column):

* **Page 1**: trip · Wh used · Wh/km · motor temperature
* **Page 2**: remaining range · max speed · average speed · controller (ESC) temperature

**C6 and AMOLED telemetry cycle** (tap the speed/telemetry zone): speed → trip → Wh used →
Wh/km → motor temp → range → max speed → average speed → FET temp → speed. The portrait
strip is narrow, so it shows pack voltage next to the battery icon (the fill carries the
percentage); landscape shows percentage and voltage.

The status area shows a Bluetooth icon (lit while a phone is connected) and a VESC-link
dot: green when telemetry is arriving, red when it has gone stale. With no link, speed and
power drop to zero and the speed is drawn greyed out - on the C6 and AMOLED boards with
`NO LINK` under it.

Faults show the decoded fault name in red: a banner across the speed area on the CYD, and
covering everything below the battery strip on the C6 and AMOLED boards.

---

## Rider profiles

Profiles are defined as a table in `config.h` - one table per link, so a CAN bike with
PAS and a UART bike without can both build from the same file. The CAN table:

```c
static constexpr RiderProfile RIDER_PROFILES[PROFILE_COUNT] = {
    //  name     colour   cur    brake  km/h    W        app_mode
    {  "OFF",    0x07E0,  0.0,   0.60f, 0.0f,   0.0f,    APP_MODE_NONE },
    {  "ECO",    0x07E0,  0.50,  0.60f, 25.0f,  250.0f,  APP_MODE_PAS },
    {  "TOUR",   0x07FF,  0.75,  0.80f, 35.0f,  400.0f,  APP_MODE_PAS },
    {  "SPORT",  0xFD20,  1.00f, 1.00f, 0.0f,   0.0f,    APP_MODE_PAS },
    {  "SPORT+", 0xF800,  1.00f, 1.00f, 0.0f,   0.0f,    APP_MODE_ADC_PAS },
};
```

`0` for speed or watts means "no limit". `PROFILE_DEFAULT_INDEX` (the first entry, `OFF`)
is what a fresh board starts on; after that the last-selected profile is remembered in
flash across dash power cycles.

`app_mode` chooses which rider inputs are live: `APP_MODE_NONE` (no rider input at all),
`APP_MODE_ADC` (throttle only), `APP_MODE_PAS` (pedal assist only) or
`APP_MODE_ADC_PAS` (both). Think hard before giving a profile you might select while
riding `APP_MODE_NONE`.

The UART table has the same five profiles with `APP_MODE_ADC` in place of the PAS modes,
since UART can only turn the throttle on (`APP_MODE_ADC`) or off (`APP_MODE_NONE`) - see
[UART](#uart). A `static_assert` fails the build if a PAS mode is put in it.

Selecting a profile sends two commands, both RAM-only:

* **`COMM_SET_MCCONF_TEMP_SETUP`** with `store = false` - the same command VESC Tool's own
  profile buttons use - carrying current scaling, a speed limit and a battery power
  limit.
* **`COMM_SET_APPCONF_NO_STORE`** for `app_mode`. Unlike the motor-config command, this
  one carries the VESC's *entire* app config, so the dash first fetches a fresh copy
  (`COMM_GET_APPCONF`), patches only the `app_to_use` byte, and sends the same blob
  straight back. It re-fetches every time rather than caching, so any other app setting
  you changed in VESC Tool since the last switch is preserved, not reverted. Rapid
  repeated switches are coalesced - the latest wins. On a firmware whose app config is a
  different length, the dash logs it and skips the `app_mode` half (limits still apply).

**Nothing is saved to the VESC's flash by a normal switch.** The limits and app mode live
in the VESC's RAM only. Your saved configuration is never written to, and a power cycle of
the controller restores it exactly. The dash re-sends the active profile automatically
whenever the controller comes back, so this is invisible in normal use.

Set `PROFILE_APPLY_TO_VESC` to `false` if you want the badge to be purely cosmetic,
and `PROFILE_FORWARD_CAN` controls whether limits also propagate to CAN-connected VESCs
(useful on dual-motor builds).

### Writing a profile to the VESC

Everything above is RAM-only by design. Holding the badge for `PROFILE_LONG_PRESS_WRITE_MS`
(10 s by default) is the one explicit exception: the dash asks the VESC to persist its
**current live configuration** with `COMM_SET_MCCONF`, so the applied profile holds across
a power cycle, with or without this dashboard attached.

It works by round trip, not by patching: the dash fetches the live config
(`COMM_GET_MCCONF` - which already reflects the profile it pushed) and sends those exact
bytes straight back. It never has to know where any individual limit lives in the blob, so
a firmware whose `mcconf` layout moved can't be silently mis-written - a reply that fails
the length check is skipped, not guessed at, and an unanswered request is retried a few
times and then given up on rather than fired late. The write is deliberately **not**
forwarded over CAN either: every controller has its own motor config, and this blob is the
addressed VESC's. The dash shows `SAVED TO VESC` only once the controller acknowledges
the write.

Three things to know before using it:

* **Flash wear.** The VESC stores its config in emulated EEPROM with a finite erase
  budget. The 10-second hold is deliberately awkward for that reason - this is not a
  per-ride gesture.
* **The write briefly stalls motor control.** Do it stopped.
* **It persists whatever is live, sentinels included.** A profile with `speed_kph` or
  `watt_max` of `0` sends "no limit" placeholders, and saving that overwrites whatever
  erpm/watt limits you had configured in VESC Tool. Save from a profile whose limits you
  actually want - not from `OFF`.

Set `PROFILE_WRITE_ENABLED` to `false` to disable the write entirely. The cycle still
commits on release; the gesture doesn't revert. `PROFILE_LONG_PRESS_WRITE_MS` must be
longer than `PROFILE_LONG_PRESS_MS` - a `static_assert` enforces it at compile time.

---

## How the dash and the BLE bridge share one CAN bus

Most telemetry needs no request at all: the VESC broadcasts `CAN_PACKET_STATUS`..`_5`
frames on its own (erpm/current/duty, Ah, Wh, temps, tachometer, voltage), and the dash
just listens. Fault code has no broadcast frame, so it is fetched with a small poll that
asks for nothing else - one CAN frame each way. Beyond that the dash asks for a few
one-off things: the battery config at startup, and the app/motor config when you switch
or save a profile.

The BLE bridge carries VESC Tool's usual framed protocol to the phone, same as ever, but
what rides between the board and the VESC is a **CAN buffer tunnel**: bytes the phone
sends get unwrapped and split into `CAN_PACKET_FILL_RX_BUFFER`/`PROCESS_RX_BUFFER` (or
`PROCESS_SHORT_BUFFER` for short commands) frames addressed to the VESC; replies get
reassembled and re-wrapped in the same framing before going back out over BLE notify.
This is the same mechanism VESC's own CAN/BLE bridge firmware (`vesc_express`) uses, so
VESC Tool sees an identical byte stream to what it would over a UART bridge - motor
detection, configuration, even firmware flashing all still pass straight through:

```
 phone --BLE(NUS)--> [RX char] --unwrap--> CAN tunnel (FILL/PROCESS) --> VESC
 phone <--notify---- [TX char] <--reframe- CAN tunnel (FILL/PROCESS) <-- VESC
                                                |
                          CAN status broadcasts +--> telemetry --> display
```

The fault-code poll uses `COMM_GET_VALUES_SELECTIVE`, tunnelled, at 10 Hz. When a phone
is connected it drops to 2 Hz (`POLL_INTERVAL_BLE_MS`) to keep bus traffic down.

Pair from the VESC Tool mobile app: scan, connect to **`CYD-VESC`**. BLE security is
**on by default** (`ENABLE_BLE_SECURITY`): pairing needs the 6-digit passkey set as
`BLE_SECURITY_PASSKEY` in `config.h`. That value is checked into the repo, so change it
for your own build.

### Over UART

A UART build has nothing to tunnel: phone frames go onto the wire as they are, and VESC
replies come back as they are. The work is keeping the two streams apart on a link with no
addresses:

* Phone bytes are reassembled into whole frames before they are written, so a dash request
  can never land in the middle of a phone frame that arrived split across BLE writes.
* Replies are routed by what the dash asked for. The dash remembers the command id of each
  request it sends that expects an answer, for up to 3 s. A reply that answers one of those
  (and, for the telemetry poll, carries the dash's own field mask) goes to the dash;
  everything else goes to the phone. Nothing goes to both, and a reply nobody is waiting
  for is dropped.
* There are no broadcasts, so the poll is all of the telemetry.

```
 phone --BLE(NUS)--> [RX char] --whole frames--> UART --> VESC
 phone <--notify---- [TX char] <--not the dash's-- UART <-- VESC
                                          |
                         dash's own replies +--> telemetry --> display
```

---

## Touch orientation

If taps land in the wrong place, flip these in `config.h` - no maths required. The CYD's
defaults are:

```c
#define TOUCH_SWAP_XY    true
#define TOUCH_INVERT_X   false
#define TOUCH_INVERT_Y   false
```

The C6 and AMOLED boards have their own defaults, chosen per layout: portrait uses the panel's
native orientation (no swap, no invert), landscape swaps the axes and inverts Y. Those are
best guesses rather than measured values - adjust if taps land in the wrong corner.

Set `TOUCH_DEBUG` to `true` to print raw and mapped coordinates while you experiment.
For resistive panels you may also want to trim `TOUCH_RAW_*_MIN/MAX` so the screen edges
are reachable.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| Blank / white screen (CYD) | wrong driver - swap `ILI9341_2_DRIVER` for `ILI9341_DRIVER` in `platformio.ini` |
| Display works, colours inverted (CYD) | same fix as above, or add `-DTFT_INVERSION_OFF` |
| Backlight on, nothing drawn (CYD) | if the panel refuses to init, add `-DUSE_HSPI_PORT` and set `TOUCH_SPI_HOST` to `VSPI` |
| C6 or AMOLED image rotated, mirrored or shifted | `SCREEN_ROTATION` / `LCD_COL_OFFSET` / `LCD_ROW_OFFSET` in `config.h` |
| AMOLED image smears or tears, serial log says `framebuffer allocation failed` | the build has no PSRAM - use the `s3_amoled164` environment as-is (it selects the 8 MB PSRAM board definition) |
| No serial output from the AMOLED board | the environment needs USB CDC on boot (`-DARDUINO_USB_CDC_ON_BOOT=1`) - it's set in `s3_amoled164`; check it survived any edits |
| Red link dot, all zeros (CAN) | CAN TX/RX swapped, wrong bitrate, wrong `VESC_CAN_TARGET_ID`, or no termination resistor on the bus |
| Red link dot, all zeros (UART) | RX/TX swapped (the usual one), the VESC's *App to Use* doesn't include UART, or its UART baud rate doesn't match `VESC_UART_BAUD` |
| Profile switch changes the limits but not throttle on/off (UART) | VESC Tool is connected through the dash - the `app_mode` change waits until it disconnects |
| UART build fails with `static assertion failed ... APP_MODE_PAS` | a profile in the UART table uses PAS, which can't run alongside the UART - use `APP_MODE_ADC` or `APP_MODE_NONE` |
| Save hold does nothing (UART) | VESC Tool is connected through the dash - a UART build refuses to save then, and the serial log says so. Disconnect it, or save from VESC Tool |
| Serial log says `[can] bus-off ...` (C6, AMOLED) | the log names the last TX error: `ACK` = nothing on the bus is acknowledging (wiring, termination, power, bitrate); `BIT`/`STUFF`/`FORM` = signal integrity. The dash retries recovery once a second until the fault is fixed |
| erpm/current/duty show up but temps, Ah, Wh, tacho and voltage stay at zero - e.g. the battery reads 0% / 0.0V and the trip never counts | CAN status message 2-5 not enabled in VESC Tool (App Settings → General). Voltage and the tachometer come only from status 5 |
| Battery shows `?` for % and voltage, range shows `--` | the VESC hasn't returned a usable battery config - see [Battery gauge](#battery-gauge). Serial prints `battery config from VESC: <n>S <x>Ah` once it lands |
| Speed shows `--`, trip never moves | the VESC hasn't returned usable geometry - see [Speed and distance](#speed-and-distance). Serial prints `geometry from VESC: <n> poles, gear <x>, wheel <d> m` once it lands |
| Speed is wildly off (and VESC Tool agrees) | the VESC's setup info is wrong or still at the skateboard defaults. Pole count is the magnet count, not pole pairs (a factor of 2); wheel diameter is in metres |
| Profile limits apply but the rider-input mode never changes | the VESC's app-config reply was the wrong size, i.e. a firmware whose app config is laid out differently from 7.0.0; the serial log says so |
| Touch dead | wrong environment for your panel (resistive vs capacitive) |
| Phone will not connect | another device is already connected (only one BLE client at a time), or the passkey doesn't match `BLE_SECURITY_PASSKEY` |
| Backlight slowly brightens and dims (CYD) | the onboard LDR sits next to the backlight and picks up its glow. Shield it with a strip of tape, or set `AUTO_BRIGHTNESS` to `false` |

Serial console at 115200 logs the detected touch controller, the link (CAN bitrate or UART
baud) and its pins, the
negotiated BLE MTU, the geometry and battery config once read, and every profile change
and save.

---

## Source map

| File | Role |
|---|---|
| `include/config.h` | everything user-tunable |
| `src/vesc_can.*` | CYD CAN transport - the only file that touches `driver/twai.h`; frame TX/RX |
| `src/vesc_can_idf5.cpp` | C6 and AMOLED CAN transport for IDF 5.x's `esp_twai` API, same interface (`include/vesc_can.h`); adds bus-off detection, recovery and TX-error logging |
| `src/vesc_uart.*` | UART transport for `VESC_LINK_UART` builds - `HardwareSerial` on UART1, byte read/write only |
| `src/vesc_protocol.*` | CRC16, framing, buffer codecs, `GET_VALUES` / CAN status parsing, frame sniffer, CAN buffer tunnel, and the hand-derived app/motor config offsets |
| `src/vesc_link.*` | owns the link (CAN or UART), decodes telemetry (broadcasts over CAN, poll replies over UART), fetches the geometry and battery config, applies profiles (limits and `app_mode`, remapped over UART), performs the flash write, carries commands/replies, keeps phone and dash traffic apart, feeds the bridge |
| `src/ble_bridge.*` | NimBLE Nordic UART Service server, chunked notifications, optional passkey security |
| `src/dash_model.*` | speed and trip/odometer (from the VESC's geometry), state of charge (from its battery config), efficiency, range |
| `src/touch_input.*` | XPT2046 / CST816 / GT911 / AXS5106L / FT3168 behind one API, plus gesture detection |
| `src/ui.*` | CYD dashboard rendering (TFT_eSPI), with dirty-region updates and LDR auto-brightness |
| `src/ui_c6.cpp` | C6 and AMOLED dashboard rendering (Arduino_GFX), portrait and landscape; the AMOLED draws through a PSRAM framebuffer |
| `src/profiles.*` | profile selection, persistence, and the save-to-VESC call |
| `src/main.cpp` | task setup and gesture wiring |
| `tools/hosttest/test_protocol.cpp` | host-side unit tests for the protocol layer |
| `tools/hosttest/sim_integration.cpp` | headless integration test against stub peripherals |
| `tools/hosttest/stub/` | minimal Arduino / TFT_eSPI / NimBLE stubs for host builds |

### Running the tests

The parts most likely to be subtly wrong are covered by tests that run on your PC, no
hardware needed. They are plain `g++` programs, not `pio test`. From `tools/hosttest/`:

```sh
# protocol layer
g++ -std=gnu++17 -Wall -Istub -I../../src -I../../include \
    test_protocol.cpp ../../src/vesc_protocol.cpp -o t && ./t

# integration, CAN link
g++ -std=gnu++17 -Wall -Istub -I../../src -I../../include -DTFT_BL=21 -DTOUCH_CAPACITIVE=1 \
    sim_integration.cpp ../../src/vesc_protocol.cpp ../../src/vesc_link.cpp \
    ../../src/dash_model.cpp ../../src/profiles.cpp -o sim && ./sim

# integration, UART link - the same file, built with the UART transport
g++ -std=gnu++17 -Wall -Istub -I../../src -I../../include -DTFT_BL=21 -DTOUCH_CAPACITIVE=1 \
    -DVESC_LINK_UART=1 \
    sim_integration.cpp ../../src/vesc_protocol.cpp ../../src/vesc_link.cpp \
    ../../src/dash_model.cpp ../../src/profiles.cpp -o sim_uart && ./sim_uart
```

They cover:

* **Protocol**: CRC16/XMODEM known-answer, frame building, resynchronisation after line
  noise, bad-CRC rejection, the firmware's non-obvious "auto" float encoding,
  `GET_VALUES` / `GET_VALUES_SELECTIVE` parsing including truncated packets, CAN status
  broadcast decoding, parsing the geometry out of `COMM_GET_MCCONF` (including rejecting a
  wrong-length reply, an odd pole count and a zero wheel diameter), and the CAN buffer
  tunnel round-tripping short and long payloads (including a bad-CRC rejection case).
* **Integration, CAN build**: the exact tunnelled payload a telemetry poll and a profile change put
  on the bus; the `app_mode` fetch-patch-resend round trip (`COMM_GET_APPCONF` →
  `COMM_SET_APPCONF_NO_STORE`, only the one byte changed, re-fetched on each switch,
  rapid switches coalesced); a CAN status broadcast decoded through the real link path
  (including one delivered a frame at a time, and one from an unrelated CAN id correctly
  ignored); a tunnelled fault-code reply decoded the same way, from a poll that asks for
  nothing else, so its reply can't overwrite a broadcast field; the `COMM_SET_MCCONF` write
  round-trip (byte-exact, written once, skipped on a wrong-size reply, and given up on
  rather than fired late when unanswered); the geometry and battery config being read
  from the same `COMM_GET_MCCONF` reply; no speed and no distance counted while the
  geometry is unknown; the battery gauge marked known once its config arrives; and
  trip / odometer / Wh-per-km / state-of-charge over a simulated 2 km ride - including the
  odometer surviving a VESC tachometer reset. The ride uses its own test geometry, so it
  doesn't depend on whatever bike `config.h` is set up for.
* **Integration, UART build**: the same transport-neutral checks (poll and profile
  payloads, the `COMM_SET_MCCONF` round trip and its ack, geometry and battery config,
  the ride), plus: throttle on/off written as `ADC and UART` / `UART` with the rest of the
  app config round-tripping untouched; an `app_mode` switch made while VESC Tool is
  connected waiting until it disconnects, and a fetch answered after it connected not
  being trusted; telemetry decoded from poll replies,
  including one arriving a byte at a time; a reply nobody is waiting for dropped; phone
  requests reaching the wire byte-for-byte, and never split by a dash poll; phone replies
  forwarded and kept out of the telemetry, including a selective reply with another mask;
  the dash's own replies kept off BLE; and no save or config traffic of the dash's own
  while VESC Tool is connected.

One known problem with the host tests as of this writing, not in the firmware itself: the
**syntax-only type check** no longer passes for `ui.cpp` and `ble_bridge.cpp`:

```sh
for f in vesc_protocol vesc_link dash_model touch_input profiles ble_bridge ui main; do
  g++ -std=gnu++17 -fsyntax-only -Wall -Wextra -Wno-unused-parameter \
      -Istub -I../../src -I../../include -DTFT_BL=21 -DTOUCH_CAPACITIVE=1 ../../src/$f.cpp
done
```

The stubs lack `analogRead` (added with LDR auto-brightness) and NimBLE's `READ_ENC` /
`WRITE_ENC` / `READ_AUTHEN` / `WRITE_AUTHEN` property flags (added with BLE security). The
other sources still pass, for both touch variants (swap in `-DTOUCH_RESISTIVE=1`).
`vesc_can.cpp`, `ui_c6.cpp` and `vesc_can_idf5.cpp` are not covered at all - there are no
`driver/twai.h`, Arduino_GFX or `esp_twai` stubs.

The stubs mirror the real library signatures but are not the real libraries, so this
catches mistakes in *this* code, not upstream API drift. It is not a substitute for a
real PlatformIO build.

---

## Credits

* [3mrotaha/pyVESC-uart](https://github.com/3mrotaha/pyVESC-uart) - telemetry field layout and scaling
* [A-Emile/VescBLEBridge](https://github.com/A-Emile/VescBLEBridge) - the BLE bridge approach and NUS UUIDs
* [TecnicoFuelCell/ComEVesc](https://github.com/TecnicoFuelCell/ComEVesc) (via SolidGeek/VescUart) - packet framing and CAN forwarding
* [vedderb/bldc](https://github.com/vedderb/bldc) - the protocol itself, `comm_can.c` in particular
* [vedderb/vesc_express](https://github.com/vedderb/vesc_express) - the official CAN/BLE bridge firmware this design follows

---

## A word of caution

This changes how a vehicle you ride behaves. Test profile switching with the wheel off
the ground before you trust it in traffic, and be aware that a long-press while riding is
a distraction - the 1.4-second hold exists precisely so it cannot happen by accident on a
bump.

---

## AI disclaimer

Much of this project's code and documentation was written with the help of AI coding
assistants, including Anthropic's Claude. A person directed and reviewed the work,
but AI-generated code can contain subtle mistakes that look plausible, and not every line
has been checked by hand. Keep that in mind when reading the source, especially the parts
that touch motor limits, the VESC's configuration or its flash.

## Disclaimer: use at your own risk

This software is provided **as is**, without warranty of any kind. You use it entirely at
your own risk.

* **It controls a vehicle.** Rider profiles change current, speed and power limits and the
  rider-input mode on a live motor controller. A bug, a misconfiguration, a bad wire or a
  dropped connection can make the bike behave unexpectedly, and that can cause a crash,
  injury or death.
* **It writes to your VESC.** The long-press save writes configuration to the VESC's flash
  and can overwrite settings you made in VESC Tool. Back up your motor and app
  configuration before you use it.
* **It involves batteries and wiring.** E-bike packs can deliver very large currents. Wiring
  mistakes can destroy the controller or the display and can start a fire.
* **It is not a certified product.** It has not been safety-tested and is not affiliated
  with or endorsed by the VESC project, Benjamin Vedder, or any board manufacturer.
* **Local laws are your responsibility.** Profiles can raise speed and power limits past
  what your local e-bike rules allow. You are responsible for staying legal on public
  roads.

The authors and contributors are not liable for any damage, injury, loss or legal
consequences that come from using, building or modifying this project. See sections 15
and 16 of the [license](LICENSE) for the formal terms.

---

## License

Copyright (C) 2026 MIH722.

This program is free software: you can redistribute it and/or modify it under the terms of
the GNU General Public License as published by the Free Software Foundation, either
version 3 of the License, or (at your option) any later version. See [LICENSE](LICENSE)
for the full text.

The GPL was chosen because the VESC protocol code this project builds on
([VescUart](https://github.com/SolidGeek/VescUart),
[ComEVesc](https://github.com/TecnicoFuelCell/ComEVesc),
[vesc_express](https://github.com/vedderb/vesc_express),
[bldc](https://github.com/vedderb/bldc)) is itself GPL-licensed. Third-party libraries pulled
in at build time (TFT_eSPI, Arduino_GFX, NimBLE-Arduino, ESP-IDF and so on) keep their
own licenses.
