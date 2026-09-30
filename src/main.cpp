// ============================================================================
//  VESC CYD Dash
//  ESP32-2432S028 ("Cheap Yellow Display") e-bike dashboard for VESC,
//  doubling as a transparent VESC Tool BLE bridge.
//
//  Core 0: link pump (CAN or UART, see VESC_LINK_UART in config.h) - reads
//          telemetry, carries requests/replies to/from the VESC, forwards
//          the BLE bridge's traffic.
//  Core 1: UI - touch, gestures, rendering at UI_FPS.
//  (On the single-core ESP32-C6 target, both still run as separate FreeRTOS
//  tasks, just time-sliced on one core rather than truly parallel - the
//  xTaskCreatePinnedToCore(..., 0) call below is still valid, it just pins
//  to the only core that exists.)
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "vesc_link.h"
#include "ble_bridge.h"
#include "dash_model.h"
#include "touch_input.h"
#include "profiles.h"
#include "ui.h"

// ---------------------------------------------------------------------------
static void ledSet(bool r, bool g, bool b) {
    if (!USE_LED_FEEDBACK) return;
    digitalWrite(RGB_LED_R_PIN, r ? LOW : HIGH);   // active low
    digitalWrite(RGB_LED_G_PIN, g ? LOW : HIGH);
    digitalWrite(RGB_LED_B_PIN, b ? LOW : HIGH);
}

// VescLink hands this every reply meant for the phone, re-framed the way
// VESC Tool expects - never the status broadcasts or the dash's own replies.
static void vescByteSink(const uint8_t *data, size_t len) {
    blebridge::notify(data, len);
}

// ---------------------------------------------------------------------------
static void linkTask(void *) {
    for (;;) {
        vescLink.loop();
        vTaskDelay(1);                 // 1 ms tick: pump the link promptly
    }
}

// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n=== VESC CYD Dash ===");

    if (USE_LED_FEEDBACK) {
        pinMode(RGB_LED_R_PIN, OUTPUT);
        pinMode(RGB_LED_G_PIN, OUTPUT);
        pinMode(RGB_LED_B_PIN, OUTPUT);
        ledSet(false, false, false);
    }

    ui::begin();
    ui::splash("VESC DASH", "looking for the controller...");

    touch::begin();
    dashStats.begin();
    profiles::begin();

    vescLink.begin();
    vescLink.setByteSink(vescByteSink);

    if (BLE_ENABLED) blebridge::begin();

    xTaskCreatePinnedToCore(linkTask, "vesc-link", 6144, nullptr, 3, nullptr, 0);

    delay(600);
    ui::forceFullRedraw();
}

// ---------------------------------------------------------------------------
//  Gesture handling
//
//  - hold the profile badge for PROFILE_LONG_PRESS_MS, release -> next profile
//  - keep holding it to PROFILE_LONG_PRESS_WRITE_MS  -> write the *current*
//    profile to the VESC's flash, and do NOT cycle
//  - hold anywhere on the speed area for PROFILE_LONG_PRESS_MS -> reset trip
//  - tap the stat area                                         -> next stat page
//    (CYD: the right-hand column, two pages; the other boards: the speed
//    zone, cycling through the telemetry fields and back to speed)
//
//  Why the badge cycle fires on release rather than the moment the threshold
//  passes (the speed area's trip reset still fires while held, unchanged):
//  the write is reachable only by continuing the same hold, so committing the
//  cycle at PROFILE_LONG_PRESS_MS would mean every write is preceded by an
//  unwanted profile change - hold 10 s to save, and you'd have saved the next
//  profile along, not the one you were riding. Deferring to release makes the
//  hold a single decision: let go early to switch, keep holding to save what
//  is already selected.
//
//  The hold sliver reflects that: it fills once towards the cycle, then
//  restarts and fills a second time towards the write, so "something else
//  happens if I keep holding" is visible rather than something you have to
//  know about.
// ---------------------------------------------------------------------------
static bool  s_longFired  = false;      // this press already committed something
static bool  s_cycleArmed = false;      // badge held past the cycle threshold
static float s_holdProgress = 0.0f;

static void handleTouch() {
    const touch::Gesture &g = touch::poll();

    if (g.justPressed) {
        s_longFired  = false;
        s_cycleArmed = false;
    }

    if (g.pressed && !s_longFired) {
        const bool onBadge = ui::inProfileBadge(g.downX, g.downY);
        const bool onSpeed = ui::inSpeedArea(g.downX, g.downY);

        if (onBadge && s_cycleArmed) {
            // Armed: the bar restarts and fills a second time, towards the
            // write. With the write disabled there is no second stage, so
            // hold it full - the cycle is ready, waiting on the release.
            s_holdProgress = PROFILE_WRITE_ENABLED
                ? (float)(g.heldMs - PROFILE_LONG_PRESS_MS) /
                  (float)(PROFILE_LONG_PRESS_WRITE_MS - PROFILE_LONG_PRESS_MS)
                : 1.0f;
        } else if (onBadge) {
            s_holdProgress = (float)g.heldMs / (float)PROFILE_LONG_PRESS_MS;
        } else {
            s_holdProgress = 0.0f;
        }

        if (g.heldMs >= PROFILE_LONG_PRESS_MS) {
            if (onBadge) {
                if (!s_cycleArmed) {
                    // Armed, not committed. Show which profile releasing now
                    // would land on - the rider is holding a badge that still
                    // reads the old name, so without this the pause between
                    // threshold and release looks like nothing happened.
                    s_cycleArmed = true;
                    ledSet(false, true, false);
                    char msg[32];
                    snprintf(msg, sizeof(msg), "RELEASE: %s",
                             RIDER_PROFILES[(profiles::index() + 1) % PROFILE_COUNT].name);
                    ui::toast(msg);
                }
                if (PROFILE_WRITE_ENABLED && g.heldMs >= PROFILE_LONG_PRESS_WRITE_MS) {
                    // Held all the way: save what is selected, and disarm the
                    // cycle so releasing from here changes nothing.
                    profiles::store();
                    char msg[32];
                    snprintf(msg, sizeof(msg), "SAVING %s...", profiles::current().name);
                    ui::toast(msg, 2500);
                    ledSet(true, true, true);
                    s_longFired    = true;
                    s_cycleArmed   = false;
                    s_holdProgress = 0.0f;
                }
            } else if (onSpeed) {
                dashStats.resetTrip();
                dashStats.persistNow();
                ui::toast("TRIP RESET");
                ledSet(false, false, true);
                s_longFired = true;
                s_holdProgress = 0.0f;
            }
        }
    }

    if (g.justReleased) {
        s_holdProgress = 0.0f;
        ledSet(false, false, false);

        // Released between the two thresholds: this is the profile cycle.
        if (s_cycleArmed) {
            s_cycleArmed = false;
            profiles::next();
            char msg[32];
            snprintf(msg, sizeof(msg), "PROFILE: %s", profiles::current().name);
            ui::toast(msg);
        }

        // short tap on the stat area -> next stat page (see above)
        if (!s_longFired && g.heldMs < 500 && ui::inStatColumn(g.downX, g.downY)) {
            ui::nextPage();
        }
    }

    if (!g.pressed) {
        s_holdProgress = 0.0f;
        s_cycleArmed   = false;   // a press lost without a release edge
    }
}

// ---------------------------------------------------------------------------
void loop() {
    static uint32_t nextFrame = 0;
    static uint32_t nextTouch = 0;
    static bool     wasAlive  = false;
    static bool     splashCleared = false;

    const uint32_t now = millis();

    if (now >= nextTouch) {           // 50 Hz is plenty for a thumb
        nextTouch = now + 20;
        handleTouch();
    }

    ui::updateAutoBrightness();       // rate-limits itself; see config.h

    if (now < nextFrame) {
        delay(2);
        return;
    }
    nextFrame = now + (1000 / UI_FPS);

    const bool alive = vescLink.isAlive();

    // A config write is fire-and-forget on the wire; the VESC echoes the
    // command back once it has actually committed to flash. Only then does
    // the rider get told it is saved.
    if (vescLink.takeStoreAck()) ui::toast("SAVED TO VESC", 2000);

    // Push the rider profile as soon as the controller answers, and again
    // whenever it comes back (a VESC reboot clears RAM-only limits).
    if (alive && !wasAlive) {
        profiles::reapply();
        ui::toast(profiles::current().name);
    }
    wasAlive = alive;

    // Show the dashboard even if nothing ever answers - a blank splash screen
    // is a worse diagnostic than a dash reading zero with a red link dot.
    if (!splashCleared && (alive || now > 4000)) {
        splashCleared = true;
        ui::forceFullRedraw();
    }

    dashStats.update(vescLink.snapshot(), alive,
                     blebridge::isConnected(), profiles::index(),
                     vescLink.batteryConfig(), vescLink.geometryConfig());

    ui::render(dashStats.model(), s_holdProgress);

    dashStats.maybePersist();
}
