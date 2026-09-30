#pragma once
#include <stdint.h>
//  ===========================================================================
//  VESC CYD Dash - user configuration
//
//  Everything you are likely to want to change lives in this one file.
//  Edit, re-flash, ride.
//  ===========================================================================

// Every board except the CYD builds on the pioarduino platform (Arduino core
// 3.x), draws with Arduino_GFX (ui_c6.cpp) and uses NimBLE-Arduino v2 - see
// platformio.ini. The board defines themselves come from there.
#if defined(BOARD_C6_LCD19) || defined(BOARD_C6_LCD147) || defined(BOARD_S3_AMOLED164)
  #define BOARD_PIOARDUINO 1
#endif

// ---------------------------------------------------------------------------
// 1. Link to the VESC: CAN bus or UART
//
//  VESC_LINK_UART picks how the dash talks to the VESC, on any board:
//    0 - CAN bus, through a TJA1050 transceiver (1a below). The default.
//    1 - UART, wired straight to the VESC's COMM port - no extra module (1b).
//  The *_uart environments in platformio.ini build with it set to 1, so a
//  board's CAN and UART firmwares are just two `pio run -e` targets, nothing
//  here to edit. Changing the default below switches every environment that
//  doesn't set it itself.
//
//  CAN is the better link when the VESC has a CAN port: telemetry arrives by
//  broadcast, phone and dash traffic are kept apart by id, and every profile
//  feature works. UART works with any VESC and needs no transceiver, but
//  gives up PAS - its profiles can only turn the throttle on or off. See 1b.
// ---------------------------------------------------------------------------
#ifndef VESC_LINK_UART
#define VESC_LINK_UART 0
#endif

// ---------------------------------------------------------------------------
//  Link pins: one TX/RX pair per board, used by whichever link is built. The
//  CAN transceiver's TXD/RXD or the VESC's COMM-port RX/TX go on the same two
//  pads - see 1a / 1b for the wiring of each. TX and RX are the dash's own:
//  its TX goes to the other side's input. Why these pads, per board:
//
//  CYD (ESP32-2432S028): only a handful of GPIOs are exposed on its two side
//  connectors:
//      P3  : GND, GPIO35 (input only), GPIO22, GPIO21 (= LCD backlight!)
//      CN1 : GND, GPIO22, GPIO27, 3V3
//  GPIO35 is input-only, so it is the natural RX. GPIO22 is the TX.
//
//  ESP32-C6-LCD-1.9 (Waveshare, touch variant): GPIO21/22 are genuinely free
//  per the board's own schematic (ESP32-C6-Touch-LCD-1.9-Schematic.pdf, via
//  docs.waveshare.com/ESP32-C6-LCD-1.9) - no LCD/SD/IMU/UART/I2C/touch use,
//  and not in the C6's 5-pin strapping set (4/5/8/9/15) or its fixed native-
//  USB pins (12/13 - USB D-/D+ on this chip regardless of board, not board-
//  specific, so not visible in any per-board pinout table). The header also
//  breaks out unrelated EXIO0-7 pins right next to them (an onboard TCA9554
//  I2C GPIO expander, not real GPIOs at all - no path from those to the TWAI
//  or UART peripherals), which is easy to wire by mistake since both are
//  silkscreened close together - double check against the schematic, not
//  just the silkscreen, if these still don't come up.
//
//  ESP32-C6-Touch-LCD-1.47 (Waveshare): unlike the 1.9" board, this one's
//  touch reset/interrupt (GPIO20/21) are genuinely wired and used - not
//  free. Its own official examples/BSP (files.waveshare.com/wiki/
//  ESP32-C6-Touch-LCD-1.47/ESP32-C6-Touch-LCD-1.47-Demo.zip) account for
//  every GPIO except 3, 5, 6, 7, 9, 10, 11. Of those, GPIO5/9 are ESP32-C6
//  strapping pins (boot mode select) - risky to let a transceiver or the
//  VESC drive at reset - and 5/9 plus 4/8/15 are the full C6 strapping set,
//  which rules out anything adjacent without checking further. GPIO7 is
//  explicitly documented as free; GPIO3 is this board's TF-card MISO
//  (freed here since this project has no use for the card - the LCD itself
//  is write-only and never used MISO) and is not a strapping pin. Both
//  clean, so: GPIO7 = TX, GPIO3 = RX.
//
//  ESP32-S3-Touch-AMOLED-1.64 (Waveshare): per its schematic (files.waveshare.
//  com/wiki/ESP32-S3-Touch-AMOLED-1.64/ESP32-S3-Touch-AMOLED-1.64-schematic.
//  pdf), header P1 breaks out GPIO1/2/3/5/6/7/8/15/16/17/18, none of them
//  used on the board. GPIO3 is an ESP32-S3 strapping pin (as are 0, 45 and
//  46), so it is out; GPIO17/18 sit side by side at the end of P1 (and are
//  the S3's default UART1 pins): GPIO17 = TX, GPIO18 = RX. (Header P2's GND
//  is the ground to use.)
// ---------------------------------------------------------------------------
#if defined(BOARD_C6_LCD19)
  #define VESC_LINK_TX_PIN    21
  #define VESC_LINK_RX_PIN    22
#elif defined(BOARD_C6_LCD147)
  #define VESC_LINK_TX_PIN    7
  #define VESC_LINK_RX_PIN    3
#elif defined(BOARD_S3_AMOLED164)
  #define VESC_LINK_TX_PIN    17
  #define VESC_LINK_RX_PIN    18
#else
  #define VESC_LINK_TX_PIN    22
  #define VESC_LINK_RX_PIN    35
#endif

// ---------------------------------------------------------------------------
// 1a. CAN link (VESC_LINK_UART 0) - TJA1050 transceiver + the ESP32's
//     built-in TWAI controller, no SPI CAN controller needed
//
//  Wiring:
//      VESC_LINK_TX_PIN -> TJA1050 TXD
//      VESC_LINK_RX_PIN <- TJA1050 RXD
//      TJA1050 VCC        -> 5V, GND -> GND
//      TJA1050 CANH/CANL  -> VESC CAN_H/CAN_L (twisted pair if the run is
//                             long)
//  Termination: the bus needs 120 ohm across CANH/CANL at EACH end. Most
//  single-VESC setups are already terminated inside the VESC, so add the
//  second 120 ohm resistor at the dash end, across the TJA1050's CANH/CANL.
//
//  VESC side: enable "CAN status message 1/2/3/4/5" in VESC Tool under
//  App Settings -> General, with a rate of at least 20-50 Hz. Without this
//  the dash only ever sees erpm/current/duty and everything else (temps,
//  Ah, Wh, tachometer, voltage) reads stuck at zero - it looks exactly like
//  a wiring fault, so don't skip it.
// ---------------------------------------------------------------------------
#define VESC_CAN_BITRATE      500000    // must match the VESC's CAN baud rate

// The VESC to show on the dash (its configured CAN controller id; 0 is the
// firmware default for a single-VESC setup).
#define VESC_CAN_TARGET_ID    119
// This dash's own address on the bus, used only for its internal telemetry/
// fault-code poll. Must not collide with any real VESC's controller id.
#define VESC_CAN_LOCAL_ID     100
// A second, separate address used only for BLE-forwarded phone traffic
// (VESC Tool's own requests - Real Time Data, config read/write, firmware
// flashing). Keeping it distinct from VESC_CAN_LOCAL_ID is what keeps the
// two request streams from colliding on the VESC's rx buffer and keeps
// phone traffic from ever being mistaken for the dash's own telemetry.
// Must differ from VESC_CAN_LOCAL_ID and not collide with any real VESC.
#define VESC_CAN_BLE_ID        101

// ---------------------------------------------------------------------------
// 1b. UART link (VESC_LINK_UART 1)
//
//  The same VESC_LINK_TX_PIN / VESC_LINK_RX_PIN pads as CAN - what changes
//  is the other end: the VESC's COMM port itself, no transceiver. Both sides
//  are 3.3 V logic, so the wires go straight across:
//      VESC_LINK_TX_PIN -> VESC RX
//      VESC_LINK_RX_PIN <- VESC TX
//      GND              -- VESC GND
//  If nothing ever answers, swap RX/TX first - it is the most common mistake.
//  The VESC's 5V pin can power the dash if it can spare what the board draws
//  with its backlight on (~150-250 mA); check your VESC's rating.
//
//  VESC side (VESC Tool, App Settings):
//    - General -> App to Use must be one that runs the UART: "UART",
//      "PPM and UART" or "ADC and UART". Nothing else does. In particular,
//      "Permanent UART" does NOT keep this port alive - it is for the VESC's
//      built-in NRF/BLE UART, a different port. That only matters until the
//      dash connects: from then on each profile sets it (see app_mode below),
//      so a PPM input does not survive the first profile the dash applies.
//    - UART -> Baudrate must match VESC_UART_BAUD (115200 is the default).
//    - Nothing to enable for telemetry: UART has no status broadcasts, so the
//      dash polls for everything (POLL_INTERVAL_MS below).
//
//  What UART gives up compared with CAN:
//    - PAS. The firmware's PAS inputs default to the COMM port's own RX/TX
//      pins, and neither PAS app runs the UART anyway. A UART build refuses
//      to compile if any profile uses APP_MODE_PAS or APP_MODE_ADC_PAS - see
//      the UART profile table in section 4.
//    - app_mode as written. Every app a profile can name stops the COMM
//      UART, which would cut the dash's own link, so over UART the dash
//      writes the app giving the same input with the UART still running:
//      APP_MODE_ADC (throttle on) becomes "ADC and UART", APP_MODE_NONE
//      (throttle off) becomes "UART". In "ADC and UART" the ADC app's
//      cruise/reverse buttons can't use the COMM port's RX/TX pins (they are
//      the UART); it reads one button on the servo/PPM pin instead.
//    - config traffic while VESC Tool is connected through the BLE bridge. A
//      UART has no addresses, so a config reply the phone asked for - maybe
//      relayed from a different VESC on the CAN bus - looks exactly like one
//      the dash asked for. So while the phone is connected, the battery/
//      geometry fetch and profile app_mode changes wait until it disconnects,
//      and the save-to-flash hold is refused. Telemetry keeps flowing, and
//      profile limits still apply straight away.
// ---------------------------------------------------------------------------
#define VESC_UART_NUM         1         // UART0 is left to the serial console
#define VESC_UART_BAUD        115200    // must match the VESC's UART baud rate

// ---------------------------------------------------------------------------
// 1c. Telemetry polling (both links)
//
//  CAN: regular telemetry (erpm/current/duty/temps/Ah/Wh/tacho/voltage)
//  arrives passively from the VESC's own CAN status broadcasts - no request
//  needed. Fault code has no broadcast frame, so it is fetched with a
//  tunnelled COMM_GET_VALUES_SELECTIVE request for that one field - a single
//  CAN frame each way - at the interval below.
//  UART: there are no broadcasts, so that same request is where *all*
//  telemetry comes from - the interval below is the dash's update rate.
// ---------------------------------------------------------------------------
#define POLL_INTERVAL_MS         100    // 10 Hz
// Keep polling (slower) while VESC Tool is connected over the BLE bridge so
// the fault-code fallback stays live without adding much bus traffic. Over
// UART this is then the whole display's update rate, since VESC Tool's own
// traffic shares the one wire.
#define POLL_WHILE_BLE_CONNECTED true
#define POLL_INTERVAL_BLE_MS     500    // 2 Hz
// Telemetry older than this counts as a lost link: the status dot turns red,
// speed and power drop to zero, and the speed is drawn greyed out.
#define TELEMETRY_STALE_MS       1500

// ---------------------------------------------------------------------------
// 2. Speed and distance
//
//    Motor pole count, gear ratio and wheel diameter - what turns ERPM and
//    tachometer counts into speed and distance - are read from the VESC
//    itself (COMM_GET_MCCONF's si_motor_poles/_gear_ratio/_wheel_diameter -
//    see vesc_link.cpp/vesc_protocol.h), never hardcoded here. They are the
//    same values the VESC uses for its own speed and distance, so the dash
//    agrees with VESC Tool by construction. Set them in VESC Tool's motor
//    config. Until that first reply lands - or on a firmware version too
//    different for the hand-derived offsets to trust - the dashboard shows
//    "--" for speed and counts no trip/odometer distance.
// ---------------------------------------------------------------------------
#define USE_IMPERIAL         false      // true -> mph and miles

// ---------------------------------------------------------------------------
// 3. Battery - used for the state-of-charge gauge
//
//    Series cell count, chemistry and pack capacity are read from the VESC
//    itself (COMM_GET_MCCONF's si_battery_cells/_type/_ah - see
//    vesc_link.cpp/vesc_protocol.h), never hardcoded here: a wrong guess
//    baked into firmware is worse than no reading at all. Until that first
//    reply lands - or on a firmware version too different for the
//    hand-derived offsets to trust - the dashboard shows "?" for battery %
//    and voltage instead of a number it can't stand behind.
// ---------------------------------------------------------------------------
// Sag compensation: SoC is estimated from  V_pack + I_in * R_internal.
// R is the resistance of the WHOLE pack (a healthy 14S Li-ion pack is roughly
// 30-60 mOhm). Measure yours, or leave the default - it only affects the
// gauge, not safety.
#define PACK_INTERNAL_R_OHM  0.045f

// ---------------------------------------------------------------------------
// 4. Rider profiles
//
//  A profile is applied with two commands, both RAM-only - a power cycle of
//  the VESC always restores your saved config, no matter what a profile did:
//    - COMM_SET_MCCONF_TEMP_SETUP (store=false), the same command VESC
//      Tool's own profile buttons use, for current_scale/brake_scale/
//      speed_kph/watt_max.
//    - COMM_SET_APPCONF_NO_STORE for app_mode. Unlike the mcconf command,
//      this one takes the VESC's *entire* app config, so the dash fetches a
//      fresh copy (COMM_GET_APPCONF) before every switch, patches just the
//      app-mode byte, and sends the same blob straight back - see
//      VescLink::sendAppMode() in vesc_link.cpp. Fetching fresh each time
//      (rather than caching) means any other app setting changed via VESC
//      Tool since the last switch is preserved, not reverted.
//      The byte offset that patch relies on is hand-verified against bldc
//      firmware release_7_00 (FW 7.0.0); re-verify against that firmware's
//      confgenerator.c if you ever upgrade.
//
//  current_scale  : fraction of the configured motor current (0.0 - 1.0)
//  brake_scale    : fraction of the configured braking current
//  speed_kph      : speed limit; use 0 for "no limit"
//  watt_max       : battery power limit in W; use 0 for "no limit"
//  app_mode       : which rider inputs are active - see AppMode below.
//                   Over UART only APP_MODE_ADC (throttle on) and
//                   APP_MODE_NONE (throttle off) are allowed (see 1b)
// ---------------------------------------------------------------------------
#define PROFILE_APPLY_TO_VESC   true    // false = the badge is display-only
#define PROFILE_FORWARD_CAN     true    // also apply to VESCs on the CAN bus
#define PROFILE_LONG_PRESS_MS   1400    // hold time to cycle profiles
#define PROFILE_COUNT           5

// ---------------------------------------------------------------------------
//  Writing a profile into the VESC's flash (the "save" hold)
//
//  Everything above is deliberately RAM-only. Keep holding the badge past
//  PROFILE_LONG_PRESS_WRITE_MS and the dashboard instead asks the VESC to
//  persist its *current live* configuration (COMM_SET_MCCONF) - the profile
//  already applied to it survives a power cycle, with or without this
//  dashboard attached.
//
//  The two holds are one gesture, not two: because the write must be
//  reachable by continuing to hold, the profile cycle can no longer fire the
//  instant PROFILE_LONG_PRESS_MS elapses - it commits on *release* instead.
//  A hold that runs all the way to the write threshold therefore never
//  cycles the profile; it saves the one already selected. That is the whole
//  point - "make what I'm riding permanent", not "switch, then save the
//  thing I switched to by accident".
//
//  Set PROFILE_WRITE_ENABLED false to disable the write entirely (the cycle
//  then still commits on release - the gesture does not revert).
//
//  Three things worth knowing before using it:
//    - flash wear. The VESC stores its config in emulated EEPROM with a
//      finite erase budget. This is a deliberate, 10-second, one-off
//      gesture for exactly that reason - it is not for every ride.
//    - the write stalls the motor control loop briefly. Do it stopped.
//    - it persists whatever is live, sentinels included. A profile with
//      speed_kph/watt_max of 0 sends "no limit" placeholders (see
//      applyProfile()), and saving that overwrites whatever erpm/watt
//      limits you had configured in VESC Tool. Save from a profile whose
//      limits you actually want, not from OFF.
// ---------------------------------------------------------------------------
#define PROFILE_WRITE_ENABLED       true
#define PROFILE_LONG_PRESS_WRITE_MS 10000   // keep holding this long to save

// The write hold is only reachable by holding *past* the cycle threshold, so
// it has to be the longer of the two. Caught at compile time rather than
// becoming an unreachable gesture (or, if inverted, a flash write on every
// profile change) that only shows up on real hardware.
static_assert(PROFILE_LONG_PRESS_WRITE_MS > PROFILE_LONG_PRESS_MS,
              "PROFILE_LONG_PRESS_WRITE_MS must be longer than PROFILE_LONG_PRESS_MS");

// Mirrors bldc firmware's app_use enum exactly (verified against bldc
// release_7_00 / FW 7.0.0 - comm/commands.c, confgenerator.c). Only the
// values a profile can pick are here; the firmware has others (PPM, NRF,
// ...). A UART build writes two of those instead - APP_USE_UART /
// APP_USE_ADC_UART in vesc_protocol.h, see 1b. APP_MODE_NONE disables rider
// input entirely (no throttle, no PAS) - think hard before assigning it to
// a profile you might select while riding.
enum AppMode : uint8_t {
    APP_MODE_NONE    = 0,    // app_use::APP_NONE     - no rider input at all
    APP_MODE_ADC     = 2,    // app_use::APP_ADC      - throttle only
    APP_MODE_PAS     = 9,    // app_use::APP_PAS      - pedal assist only
    APP_MODE_ADC_PAS = 10,   // app_use::APP_ADC_PAS  - throttle + PAS
};

struct RiderProfile {
    const char *name;
    uint16_t    color;          // RGB565 accent
    float       current_scale;
    float       brake_scale;
    float       speed_kph;      // 0 = unlimited
    float       watt_max;       // 0 = unlimited
    AppMode     app_mode;
};

// Tune these to taste. Order = the order the long-press cycles through.
// There is one table per link, so a CAN bike with PAS and a UART bike without
// can both build from this one file. constexpr, not just const, so the UART
// table can be checked at compile time (below).
#if !VESC_LINK_UART
// CAN: any app_mode - edit per profile if you want one that changes rider
// input too.
static constexpr RiderProfile RIDER_PROFILES[PROFILE_COUNT] = {
    //  name     colour   cur    brake  km/h    W        app_mode
    {  "OFF",    0x07E0,  0.0,   0.60f, 0.0f,   0.0f,    APP_MODE_NONE },
    {  "ECO",    0x07E0,  0.50,  0.60f, 25.0f,  250.0f,  APP_MODE_PAS },
    {  "TOUR",   0x07FF,  0.75,  0.80f, 35.0f,  400.0f,  APP_MODE_PAS },
    {  "SPORT",  0xFD20,  1.00f, 1.00f, 0.0f,   0.0f,    APP_MODE_PAS },
    {  "SPORT+", 0xF800,  1.00f, 1.00f, 0.0f,   0.0f,    APP_MODE_ADC_PAS },
};
#else
// UART: throttle on (APP_MODE_ADC) or off (APP_MODE_NONE) only - no PAS over
// UART, see section 1b. The dash turns these into "ADC and UART" / "UART" so
// the link survives the switch.
static constexpr RiderProfile RIDER_PROFILES[PROFILE_COUNT] = {
    //  name     colour   cur    brake  km/h    W        app_mode
    {  "OFF",    0x07E0,  0.0,   0.60f, 0.0f,   0.0f,    APP_MODE_NONE },
    {  "ECO",    0x07E0,  0.50,  0.60f, 25.0f,  250.0f,  APP_MODE_ADC },
    {  "TOUR",   0x07FF,  0.75,  0.80f, 35.0f,  400.0f,  APP_MODE_ADC },
    {  "SPORT",  0xFD20,  1.00f, 1.00f, 0.0f,   0.0f,    APP_MODE_ADC },
    {  "SPORT+", 0xF800,  1.00f, 1.00f, 0.0f,   0.0f,    APP_MODE_ADC },
};

// PAS can't run alongside the COMM-port UART, so a UART profile asking for
// it could never be honoured - fail the build rather than find out on the
// bike. Recursive, single-return: the CYD builds compile as C++11 (the
// platform's own -std=gnu++11 lands after ours), which allows nothing else
// in a constexpr function.
constexpr bool uartProfilesUsePas(int i = 0) {
    return i < PROFILE_COUNT &&
           (RIDER_PROFILES[i].app_mode == APP_MODE_PAS ||
            RIDER_PROFILES[i].app_mode == APP_MODE_ADC_PAS ||
            uartProfilesUsePas(i + 1));
}
static_assert(!uartProfilesUsePas(),
              "UART build: a rider profile uses APP_MODE_PAS or APP_MODE_ADC_PAS. "
              "PAS can't share the VESC's COMM port with the UART link - use "
              "APP_MODE_ADC (throttle on) or APP_MODE_NONE (throttle off). "
              "See config.h section 1b.");
#endif
#define PROFILE_DEFAULT_INDEX   0

// ---------------------------------------------------------------------------
// 5. BLE bridge (VESC Tool compatible - Nordic UART Service)
//    Leave the UUIDs alone; VESC Tool looks for exactly these.
// ---------------------------------------------------------------------------
#define BLE_ENABLED          true
#define BLE_DEVICE_NAME      "CYD-VESC"
#define BLE_TX_POWER         ESP_PWR_LVL_P9
#define BLE_MTU              256
#define VESC_SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define VESC_CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define VESC_CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
// Requires a 6-digit passkey before anything can connect. Comment out to
// turn that off (bonding only, no passkey).
#define ENABLE_BLE_SECURITY
#define BLE_SECURITY_PASSKEY 384752

// ---------------------------------------------------------------------------
// 6. Display / touch
// ---------------------------------------------------------------------------
// Portrait vs landscape UI, for every board ui_c6.cpp draws (the two C6
// boards and the S3 AMOLED - not the CYD). All three panels are physically
// portrait-native, so this just picks which way this firmware treats them -
// it does not need different wiring. Both orientations have the same
// zones: battery strip, a middle speed/telemetry zone (tap to cycle
// through the rest of the telemetry, hold to reset trip), profile band
// (hold to change), and a power meter.
//   false:            battery top-left, BLE + VESC-link status top-right,
//                     speed centred in the middle band, profile
//                     bottom-left, power meter bottom-right.
//   true (default):   battery (voltage only, no %) top-left, BLE +
//                     VESC-link status top-right, speed upper-middle,
//                     power meter below that, profile band along the
//                     bottom.
#define UI_LAYOUT_PORTRAIT true

#if defined(BOARD_C6_LCD19)
  // Waveshare ESP32-C6-LCD-1.9: ST7789V2, 170x320 native. This rotation
  // value is for Arduino_GFX (ui_c6.cpp) - a different numbering convention
  // than TFT_eSPI's below - and, like the panel's GRAM column/row offsets
  // below, has not been verified on real hardware yet. If the image
  // comes up rotated, mirrored, or offset, this is the first thing to
  // adjust (both rotation values below, not just the landscape one).
  #if UI_LAYOUT_PORTRAIT
    #define SCREEN_ROTATION    0
    #define SCREEN_W           170
    #define SCREEN_H           320
  #else
    #define SCREEN_ROTATION    1
    #define SCREEN_W           320
    #define SCREEN_H           170
  #endif
  #define LCD_COL_OFFSET       35     // GRAM offset, both col1/col2 (ui_c6.cpp)
  #define LCD_ROW_OFFSET       0
  #define LCD_DC_PIN           6
  #define LCD_CS_PIN           7
  #define LCD_SCK_PIN          5
  #define LCD_MOSI_PIN         4
  #define LCD_RST_PIN          14
  #define LCD_BL_PIN           15
#elif defined(BOARD_C6_LCD147)
  // Waveshare ESP32-C6-Touch-LCD-1.47: JD9853, 172x320 native. Same
  // rotation-numbering caveat as the 1.9 board above - unverified on real
  // hardware. Unlike the 1.9 board's ST7789V2, JD9853 needs an extra vendor
  // register-tuning sequence beyond what Arduino_ST7789's generic init
  // does - see the BOARD_C6_LCD147 branch in ui_c6.cpp, copied verbatim
  // from Waveshare's own example (files.waveshare.com/wiki/
  // ESP32-C6-Touch-LCD-1.47/ESP32-C6-Touch-LCD-1.47-Demo.zip,
  // Arduino/examples/01_gfx_helloworld).
  #if UI_LAYOUT_PORTRAIT
    #define SCREEN_ROTATION    0
    #define SCREEN_W           172
    #define SCREEN_H           320
  #else
    #define SCREEN_ROTATION    1
    #define SCREEN_W           320
    #define SCREEN_H           172
  #endif
  #define LCD_COL_OFFSET       34     // GRAM offset, both col1/col2 (ui_c6.cpp)
  #define LCD_ROW_OFFSET       0
  #define LCD_DC_PIN           15
  #define LCD_CS_PIN           14
  #define LCD_SCK_PIN          1
  #define LCD_MOSI_PIN         2
  #define LCD_RST_PIN          22
  #define LCD_BL_PIN           23
#elif defined(BOARD_S3_AMOLED164)
  // Waveshare ESP32-S3-Touch-AMOLED-1.64: CO5300 AMOLED, 280x456 native,
  // over QSPI. Pins, the 20-column GRAM offset and the 40 MHz clock are from
  // Waveshare's own example (files.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-
  // 1.64/ESP32-S3-Touch-AMOLED-1.64-Demo.zip, Arduino/examples/06_LVGL_Test),
  // unverified on real hardware. There is no backlight pin - an AMOLED lights
  // its own pixels, and brightness is a display command instead (ui_c6.cpp).
  // The CO5300 can't rotate in hardware either; ui_c6.cpp draws into a
  // framebuffer that does, so both layouts work.
  #if UI_LAYOUT_PORTRAIT
    #define SCREEN_ROTATION    0
    #define SCREEN_W           280
    #define SCREEN_H           456
  #else
    #define SCREEN_ROTATION    1
    #define SCREEN_W           456
    #define SCREEN_H           280
  #endif
  #define LCD_COL_OFFSET       20     // GRAM offset (ui_c6.cpp)
  #define LCD_ROW_OFFSET       0
  #define LCD_CS_PIN           9
  #define LCD_SCK_PIN          10
  #define LCD_D0_PIN           11
  #define LCD_D1_PIN           12
  #define LCD_D2_PIN           13
  #define LCD_D3_PIN           14
  #define LCD_RST_PIN          21
#else
  #define SCREEN_ROTATION      1          // 1 or 3 = landscape 320x240
  #define SCREEN_W             320
  #define SCREEN_H             240
#endif
// On the S3 AMOLED this is the panel's own pixel brightness. Its static
// elements (profile band, battery strip) can burn in over long rides at high
// brightness - lower it there if that matters to you.
#define BACKLIGHT_PCT        90         // 5..100 - fixed brightness when AUTO_BRIGHTNESS is false
#define UI_FPS               20

#if defined(BOARD_PIOARDUINO)
// No onboard LDR on the C6 boards or the S3 AMOLED - the analogue pin they
// do use is BAT_ADC (battery voltage: GPIO0 on the C6s, GPIO4 on the S3),
// not a light sensor. Brightness stays fixed at BACKLIGHT_PCT.
#define AUTO_BRIGHTNESS false
#else
// Auto brightness: this board revision has an onboard LDR (light-dependent
// resistor) at GPIO34, right next to the display. Community-reported
// behaviour (this board does not document it): a LOWER raw analogRead()
// means BRIGHTER light - normal room light (and the display's own backlight
// spilling onto the sensor) saturates the reading near 0, while covering the
// sensor reads roughly 1000-1700. Log raw LDR_PIN readings over serial for
// your unit and adjust LDR_ADC_BRIGHT/LDR_ADC_DARK if the range looks off.
//
// Because the sensor sits this close to the backlight, its reading is not
// purely ambient light - dimming the backlight reduces the spill the sensor
// sees, which can make it read "darker" and brighten again: a feedback loop.
// updateAutoBrightness()'s heavy smoothing (ui.cpp) exists specifically to
// keep that from visibly hunting. If you still see slow flicker, shield the
// sensor from the backlight (a strip of tape / a small baffle) so it reads
// true ambient light instead.
#define AUTO_BRIGHTNESS            true
#define LDR_PIN                    34
#define LDR_ADC_BRIGHT             50      // raw reading at/below this -> BACKLIGHT_MAX_PCT
#define LDR_ADC_DARK               1200    // raw reading at/above this -> BACKLIGHT_MIN_PCT
#define BACKLIGHT_MIN_PCT          15      // floor - never dimmer than this
#define BACKLIGHT_MAX_PCT          100     // ceiling
#define AUTO_BRIGHTNESS_UPDATE_MS  1000    // how often to re-sample the LDR
#endif

// Full-scale ends of the power bar, in watts.
#define POWER_BAR_MAX_W      1000.0f
#define POWER_BAR_REGEN_W    1000.0f
// Warn colours
#define TEMP_WARN_FET_C      70.0f
#define TEMP_WARN_MOTOR_C    80.0f

// Touch orientation fixes. If taps land in the wrong corner, flip these.
// Defaults are for SCREEN_ROTATION 1 (landscape) with the panel in its
// native portrait - unverified for the C6 boards, which have a different
// native panel size; if taps land wrong there, this is the place to fix it.
// UI_LAYOUT_PORTRAIT uses SCREEN_ROTATION 0 (the panel's own native
// orientation, no rotation applied), where native panel coordinates and
// screen coordinates are the same thing - so the natural starting guess is
// no swap/invert at all, unlike landscape's 90-degree rotation. Equally
// unverified; adjust here if needed.
#if defined(BOARD_PIOARDUINO)
  #define TOUCH_INVERT_X       false
#if UI_LAYOUT_PORTRAIT
  #define TOUCH_SWAP_XY        false
  #define TOUCH_INVERT_Y       false
#else
  #define TOUCH_SWAP_XY        true
  #define TOUCH_INVERT_Y       true
#endif
#else 
  #define TOUCH_SWAP_XY        true
  #define TOUCH_INVERT_X       false
  #define TOUCH_INVERT_Y       false
#endif

#if defined(BOARD_C6_LCD19)
  // Native panel geometry (portrait, before rotation) for touch_input.cpp's
  // coordinate mapping.
  #define TOUCH_PANEL_W        170
  #define TOUCH_PANEL_H        320

  // Capacitive CST8xx @ 0x15 on I2C, shared with the onboard QMI8658 IMU -
  // same protocol family the CST816/820 branch below already speaks (see
  // touch_input.cpp). TP_RST/TP_INT are silkscreened to GPIO21/22 but marked
  // NC (not actually wired) on this board's own pinout diagram, hence -1.
  #define TOUCH_SDA_PIN        18
  #define TOUCH_SCL_PIN        8
  #define TOUCH_RST_PIN        -1
  #define TOUCH_INT_PIN        -1
  #define TOUCH_I2C_HZ         400000
#elif defined(BOARD_C6_LCD147)
  // Native panel geometry (portrait, before rotation) for touch_input.cpp's
  // coordinate mapping.
  #define TOUCH_PANEL_W        172
  #define TOUCH_PANEL_H        320

  // AXS5106L @ 0x63 on I2C - a genuinely different chip/protocol from the
  // CST8xx family (see the BOARD_C6_LCD147 branch in touch_input.cpp),
  // confirmed from Waveshare's own Arduino driver source (esp_lcd_touch_
  // axs5106l.cpp in the demo zip referenced above). Unlike the 1.9 board,
  // TOUCH_RST_PIN here is genuinely wired and used (their driver toggles it
  // on init) - only the interrupt is left unused, same "poll, don't attach
  // an interrupt" choice made everywhere else in this file.
  #define TOUCH_SDA_PIN        18
  #define TOUCH_SCL_PIN        19
  #define TOUCH_RST_PIN        20
  #define TOUCH_INT_PIN        -1
  #define TOUCH_I2C_HZ         400000
#elif defined(BOARD_S3_AMOLED164)
  // Native panel geometry (portrait, before rotation) for touch_input.cpp's
  // coordinate mapping.
  #define TOUCH_PANEL_W        280
  #define TOUCH_PANEL_H        456

  // FT3168 @ 0x38 on I2C, shared with the onboard QMI8658 IMU. FocalTech
  // register layout - see the BOARD_S3_AMOLED164 branch in touch_input.cpp.
  // Its reset and interrupt lines aren't wired to the ESP32 (Waveshare's
  // demo never touches them either), and 300 kHz is the rate that demo uses.
  #define TOUCH_SDA_PIN        47
  #define TOUCH_SCL_PIN        48
  #define TOUCH_RST_PIN        -1
  #define TOUCH_INT_PIN        -1
  #define TOUCH_I2C_HZ         300000
#else
  // Native panel geometry (portrait, before rotation) for touch_input.cpp's
  // coordinate mapping.
  #define TOUCH_PANEL_W        240
  #define TOUCH_PANEL_H        320

  #if defined(TOUCH_RESISTIVE)
    // XPT2046 on its own SPI bus (the TFT keeps VSPI).
    #define TOUCH_SPI_HOST     HSPI
    #define TOUCH_CLK_PIN      25
    #define TOUCH_MOSI_PIN     32
    #define TOUCH_MISO_PIN     39
    #define TOUCH_CS_PIN       33
    #define TOUCH_IRQ_PIN      36
    // Raw ADC range -> panel edges. Run the sketch, watch the serial log
    // with TOUCH_DEBUG on, and adjust if the edges are unreachable.
    #define TOUCH_RAW_X_MIN    300
    #define TOUCH_RAW_X_MAX    3800
    #define TOUCH_RAW_Y_MIN    300
    #define TOUCH_RAW_Y_MAX    3800
    #define TOUCH_PRESSURE_MIN 300
  #else
    // Capacitive panel (CST816/CST820 @0x15, or GT911 @0x5D/0x14) on I2C.
    // These are the pins Sunton uses on the capacitive members of this
    // family; check your board's silkscreen if the controller is not
    // detected.
    #define TOUCH_SDA_PIN      33
    #define TOUCH_SCL_PIN      32
    #define TOUCH_RST_PIN      25
    // The panel INT line sits on GPIO21 on several of these boards - which
    // is also the LCD backlight on the resistive ones. We poll rather than
    // use the interrupt, so leave this at -1 unless you have checked your
    // schematic.
    #define TOUCH_INT_PIN      -1
    #define TOUCH_I2C_HZ       400000
  #endif
#endif

#define TOUCH_DEBUG          false      // log raw + mapped coordinates

// ---------------------------------------------------------------------------
// 7. Misc
// ---------------------------------------------------------------------------
#define ODO_SAVE_INTERVAL_MS 60000      // flash wear: persist odometer once a minute
#if defined(BOARD_C6_LCD19)
  // No simple 3-wire RGB LED on this board to drive this way (the touch
  // variant does not populate the WS2812 the non-touch variant has, and a
  // WS2812 needs its own single-wire protocol regardless) - these pins are
  // never touched since USE_LED_FEEDBACK is false, but must still name
  // something unclaimed to compile: GPIO3/GPIO23 are free/unused spares.
  #define RGB_LED_R_PIN       3
  #define RGB_LED_G_PIN       23
  #define RGB_LED_B_PIN       23
  #define USE_LED_FEEDBACK    false
#elif defined(BOARD_C6_LCD147)
  // No 3-wire RGB LED on this board either, and its two genuinely spare
  // pins (3, 7) are both spoken for by the VESC link above - these
  // placeholders are never touched since USE_LED_FEEDBACK is false, so any
  // already-claimed pin is fine to name here; reusing UART0's (16/17)
  // rather than inventing a third meaning for a pin.
  #define RGB_LED_R_PIN       16
  #define RGB_LED_G_PIN       17
  #define RGB_LED_B_PIN       17
  #define USE_LED_FEEDBACK    false
#elif defined(BOARD_S3_AMOLED164)
  // No GPIO-driven LED on this board (its two LEDs are hardwired to power and
  // charging). Placeholders only, never touched while USE_LED_FEEDBACK is
  // false: GPIO42 is connected to nothing and isn't on either header.
  #define RGB_LED_R_PIN       42
  #define RGB_LED_G_PIN       42
  #define RGB_LED_B_PIN       42
  #define USE_LED_FEEDBACK    false
#else
  #define RGB_LED_R_PIN       4          // on-board RGB LED (active LOW)
  #define RGB_LED_G_PIN       16
  #define RGB_LED_B_PIN       17
  #define USE_LED_FEEDBACK    false       // blink on profile change / fault
#endif
