#pragma once
#include <Arduino.h>
#include "vesc_protocol.h"
#include "config.h"

// ============================================================================
//  Turns raw VESC telemetry into the numbers a rider actually reads:
//  speed, state of charge, trip, odometer, efficiency, remaining range.
// ============================================================================

struct DashModel {
    // live
    float   speed        = 0;      // km/h or mph, per USE_IMPERIAL
    float   voltage      = 0;
    float   powerW       = 0;      // battery-side, negative = regenerating
    float   motorCurrent = 0;
    float   batteryPct   = 0;
    float   tempFet      = 0;
    float   tempMotor    = 0;
    uint8_t faultCode    = 0;
    // False until the VESC's own battery config (cell count + a chemistry
    // this dash has a SoC curve for) has been read - there is no hardcoded
    // guess to fall back to, so the UI should render batteryPct/voltage as
    // "?" rather than a number while this is false. See vesc_link.h's
    // batteryConfig().
    bool    battKnown    = false;
    // Same idea for speed and distance: false until the VESC's own motor pole
    // count, gear ratio and wheel diameter have been read. While false, speed
    // renders as "--" and no trip/odometer distance is counted. See
    // vesc_link.h's geometryConfig().
    bool    geomKnown    = false;

    // ride
    float   tripDist     = 0;      // km or mi
    float   odoDist      = 0;
    float   whUsed       = 0;
    float   whPerDist    = 0;      // Wh/km or Wh/mi
    float   rangeLeft    = 0;      // km or mi, 0 if unknown
    float   maxSpeed     = 0;
    float   avgSpeed     = 0;

    // status
    bool    linkOk       = false;
    bool    bleConnected = false;
    uint8_t profileIndex = 0;
};

class DashStats {
public:
    void begin();                              // loads odometer from NVS
    // `batt` is optional: pass vescLink.batteryConfig() to use the VESC's own
    // si_battery_cells/_ah/_type. There is no hardcoded fallback - an invalid
    // (default) BatteryConfig, or one whose chemistry doesn't map to a curve
    // this dash knows, leaves DashModel::battKnown false and batteryPct/
    // rangeLeft unset instead of guessing.
    // `geom` likewise: pass vescLink.geometryConfig() for the VESC's own
    // si_motor_poles/_gear_ratio/_wheel_diameter. An invalid (default) one
    // leaves DashModel::geomKnown false, speed at 0 and distance uncounted.
    void update(const vesc::Telemetry &t, bool linkOk, bool ble, uint8_t profileIdx,
                const vesc::BatteryConfig &batt = vesc::BatteryConfig(),
                const vesc::GeometryConfig &geom = vesc::GeometryConfig());
    void resetTrip();
    void maybePersist();                       // rate-limited NVS write
    void persistNow();

    const DashModel &model() const { return _m; }

    static const char *distUnit()  { return USE_IMPERIAL ? "mi"   : "km"; }
    static const char *speedUnit() { return USE_IMPERIAL ? "mph"  : "km/h"; }

private:
    float metersFromTacho(int32_t tacho, const vesc::GeometryConfig &g) const;
    float batterySoC(float packV, float currentIn, uint8_t cellsS, bool liIon) const;

    DashModel _m;
    int32_t   _tachoRef      = 0;     // tachometer reading at trip start
    bool      _tachoRefValid = false;
    float     _odoMeters     = 0;     // persisted
    float     _lastTripM     = 0;
    float     _whRef         = 0;
    bool      _whRefValid    = false;
    uint32_t  _movingMs      = 0;
    uint32_t  _lastUpdateMs  = 0;
    uint32_t  _lastSaveMs    = 0;
    float     _speedFilt     = 0;
    float     _powerFilt     = 0;
    float     _socFilt       = -1;
};

extern DashStats dashStats;
