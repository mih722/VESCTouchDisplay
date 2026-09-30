#include "dash_model.h"
#include <Preferences.h>
#include <math.h>

DashStats dashStats;

static Preferences s_prefs;
static const float KM_PER_MI = 1.609344f;

// ---------------------------------------------------------------------------
//  Open-circuit voltage -> state of charge, per cell.
//  Coarse but honest; a coulomb counter would drift, and riders mostly want
//  "how much is left" to the nearest 5%.
// ---------------------------------------------------------------------------
struct SocPoint { float v; float pct; };

static const SocPoint LIION_CURVE[] = {
    {3.20f,   0}, {3.45f,   5}, {3.55f,  10}, {3.65f,  20}, {3.71f,  30},
    {3.76f,  40}, {3.81f,  50}, {3.86f,  60}, {3.92f,  70}, {3.98f,  80},
    {4.06f,  90}, {4.15f,  97}, {4.20f, 100},
};

static const SocPoint LIFEPO4_CURVE[] = {
    {2.60f,   0}, {2.90f,   5}, {3.10f,  10}, {3.20f,  20}, {3.23f,  30},
    {3.26f,  40}, {3.28f,  50}, {3.30f,  60}, {3.32f,  70}, {3.34f,  80},
    {3.37f,  90}, {3.45f,  97}, {3.60f, 100},
};

static float interpSoc(const SocPoint *c, size_t n, float v) {
    if (v <= c[0].v)     return 0.0f;
    if (v >= c[n - 1].v) return 100.0f;
    for (size_t i = 1; i < n; i++) {
        if (v < c[i].v) {
            const float span = c[i].v - c[i - 1].v;
            const float f = (span > 0) ? (v - c[i - 1].v) / span : 0.0f;
            return c[i - 1].pct + f * (c[i].pct - c[i - 1].pct);
        }
    }
    return 100.0f;
}

// ---------------------------------------------------------------------------
void DashStats::begin() {
    s_prefs.begin("vescdash", false);
    _odoMeters = s_prefs.getFloat("odo_m", 0.0f);
    Serial.printf("[stats] odometer restored: %.2f km\n", _odoMeters / 1000.0f);
}

float DashStats::metersFromTacho(int32_t tacho, const vesc::GeometryConfig &g) const {
    // Matches mc_interface_get_distance() in the bldc firmware:
    //   distance = tacho * (pi * D) / (3 * poles * gear_ratio)
    // The tachometer counts commutation steps, six per electrical revolution.
    const float scale = (g.wheelDiameterM * (float)M_PI) /
                        (3.0f * (float)g.motorPoles * g.gearRatio);
    return (float)tacho * scale;
}

float DashStats::batterySoC(float packV, float currentIn, uint8_t cellsS, bool liIon) const {
    if (packV < 1.0f) return 0.0f;

    // Compensate for sag so the gauge does not collapse under throttle.
    // PACK_INTERNAL_R_OHM is the resistance of the whole pack, so the drop is
    // I * R for the pack as a unit - do NOT scale it by the cell count.
    float compensated = packV + currentIn * PACK_INTERNAL_R_OHM;

    // A bad resistance estimate at high current could otherwise push the
    // estimate off the top of the curve; clamp to a physically sane ceiling.
    const float cellMax = liIon ? 4.25f : 3.70f;
    compensated = min(compensated, cellMax * (float)cellsS);

    const float perCell = compensated / (float)cellsS;

    if (liIon) {
        return interpSoc(LIION_CURVE, sizeof(LIION_CURVE) / sizeof(SocPoint), perCell);
    }
    return interpSoc(LIFEPO4_CURVE, sizeof(LIFEPO4_CURVE) / sizeof(SocPoint), perCell);
}

// ---------------------------------------------------------------------------
void DashStats::update(const vesc::Telemetry &t, bool linkOk, bool ble, uint8_t profileIdx,
                       const vesc::BatteryConfig &batt, const vesc::GeometryConfig &geom) {
    const uint32_t now = millis();
    const float dt = (_lastUpdateMs == 0) ? 0.0f : (now - _lastUpdateMs) / 1000.0f;
    _lastUpdateMs = now;

    _m.linkOk       = linkOk;
    _m.bleConnected = ble;
    _m.profileIndex = profileIdx;

    // There is no hardcoded cell-count/capacity/chemistry to fall back to:
    // a wrong guess baked into firmware is worse than an honest "unknown"
    // gauge. `chemistry >= 0` also gates this, not just `valid`, since a SoC
    // curve can't be picked without knowing which one to use (see
    // BatteryConfig's comment in vesc_protocol.h).
    const bool battKnown = batt.valid && batt.chemistry >= 0;
    _m.battKnown = battKnown;
    // Same for geometry: no pole count/gear ratio/wheel diameter of our own
    // to guess with, so without the VESC's there is no speed or distance.
    const bool geomKnown = geom.valid;
    _m.geomKnown = geomKnown;

    if (!linkOk) {
        _m.speed  = 0;
        _m.powerW = 0;
        return;
    }

    // ---- speed ------------------------------------------------------------
    if (geomKnown) {
        // erpm -> mechanical rpm -> wheel rpm -> m/s, as the firmware's own
        // mc_interface_get_speed() does it
        const float mechRpm  = t.erpm / ((float)geom.motorPoles / 2.0f);
        const float wheelRpm = mechRpm / geom.gearRatio;
        const float ms       = wheelRpm * (geom.wheelDiameterM * (float)M_PI) / 60.0f;
        float kph = fabsf(ms) * 3.6f;

        // light smoothing: FOC erpm is noisy at walking pace
        _speedFilt += (kph - _speedFilt) * 0.35f;
        kph = _speedFilt;

        _m.speed = USE_IMPERIAL ? kph / KM_PER_MI : kph;
    } else {
        _speedFilt = 0;
        _m.speed   = 0;     // rendered as "--" - see DashModel::geomKnown
    }
    _m.voltage = t.voltage;

    const float p = t.voltage * t.currentIn;
    _powerFilt += (p - _powerFilt) * 0.30f;
    _m.powerW       = _powerFilt;
    _m.motorCurrent = t.currentMotor;
    _m.tempFet      = t.tempFet;
    _m.tempMotor    = t.tempMotor;
    _m.faultCode    = t.faultCode;

    // ---- state of charge --------------------------------------------------
    if (battKnown) {
        const float soc = batterySoC(t.voltage, t.currentIn, batt.cells, (bool)batt.chemistry);
        if (_socFilt < 0) _socFilt = soc;
        _socFilt += (soc - _socFilt) * 0.05f;  // heavy filter: gauges must not twitch
        _m.batteryPct = constrain(_socFilt, 0.0f, 100.0f);
    } else {
        _socFilt = -1;   // so the filter starts clean if battKnown ever flips true
    }

    // ---- distance ---------------------------------------------------------
    // Tacho counts can't become metres without the geometry, so nothing is
    // counted until it is known - and the trip baseline isn't taken until
    // then either, so trip and odometer start counting at the same moment.
    // In practice that is a second or two after boot, before anyone moves.
    if (geomKnown) {
        // The VESC's tachometer resets when it reboots; re-baseline instead
        // of logging a phantom trip of several kilometres.
        if (!_tachoRefValid || t.tachometerAbs < _tachoRef) {
            _tachoRef = t.tachometerAbs;
            _tachoRefValid = true;
            _lastTripM = 0;
        }
        const float tripM = metersFromTacho(t.tachometerAbs - _tachoRef, geom);
        const float deltaM = tripM - _lastTripM;
        if (deltaM > 0 && deltaM < 500.0f) {   // ignore implausible jumps
            _odoMeters += deltaM;
        }
        _lastTripM = tripM;

        _m.tripDist = USE_IMPERIAL ? tripM / 1000.0f / KM_PER_MI : tripM / 1000.0f;
    }
    _m.odoDist = USE_IMPERIAL ? _odoMeters / 1000.0f / KM_PER_MI : _odoMeters / 1000.0f;

    // ---- energy -----------------------------------------------------------
    const float whNet = t.wattHours - t.wattHoursCharged;
    if (!_whRefValid) { _whRef = whNet; _whRefValid = true; }
    if (whNet < _whRef) _whRef = whNet;        // VESC counters were reset
    _m.whUsed = whNet - _whRef;

    if (_m.tripDist > 0.05f) {
        _m.whPerDist = _m.whUsed / _m.tripDist;
    }

    // ---- range ------------------------------------------------------------
    if (battKnown) {
        const float packWh = batt.ah * (float)batt.cells * (batt.chemistry ? 3.7f : 3.2f);
        const float whLeft = packWh * _m.batteryPct / 100.0f;
        _m.rangeLeft = (_m.whPerDist > 1.0f) ? whLeft / _m.whPerDist : 0.0f;
    } else {
        _m.rangeLeft = 0.0f;   // already renders as "--" in the UI
    }

    // ---- speed statistics -------------------------------------------------
    if (_m.speed > _m.maxSpeed) _m.maxSpeed = _m.speed;
    if (_m.speed > 1.0f && dt > 0 && dt < 1.0f) {
        _movingMs += (uint32_t)(dt * 1000.0f);
        const float hours = _movingMs / 3600000.0f;
        if (hours > 0.0002f) _m.avgSpeed = _m.tripDist / hours;
    }
}

// ---------------------------------------------------------------------------
void DashStats::resetTrip() {
    _tachoRefValid = false;
    _whRefValid    = false;
    _lastTripM     = 0;
    _movingMs      = 0;
    _m.tripDist    = 0;
    _m.whUsed      = 0;
    _m.whPerDist   = 0;
    _m.maxSpeed    = 0;
    _m.avgSpeed    = 0;
}

void DashStats::maybePersist() {
    if (millis() - _lastSaveMs >= ODO_SAVE_INTERVAL_MS) persistNow();
}

void DashStats::persistNow() {
    _lastSaveMs = millis();
    s_prefs.putFloat("odo_m", _odoMeters);
}
