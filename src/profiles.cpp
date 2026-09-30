#include "profiles.h"
#include "vesc_link.h"
#include <Preferences.h>

namespace profiles {

static Preferences s_prefs;
static uint8_t     s_index = PROFILE_DEFAULT_INDEX;

void begin() {
    s_prefs.begin("vescprof", false);
    s_index = s_prefs.getUChar("idx", PROFILE_DEFAULT_INDEX);
    if (s_index >= PROFILE_COUNT) s_index = PROFILE_DEFAULT_INDEX;
    Serial.printf("[profile] restored: %s\n", RIDER_PROFILES[s_index].name);
}

uint8_t index() { return s_index; }

const RiderProfile &current() { return RIDER_PROFILES[s_index]; }

void set(uint8_t idx) {
    s_index = (uint8_t)(idx % PROFILE_COUNT);
    s_prefs.putUChar("idx", s_index);
    vescLink.applyProfile(RIDER_PROFILES[s_index]);
}

void next() { set((uint8_t)(s_index + 1)); }

void reapply() { vescLink.applyProfile(RIDER_PROFILES[s_index]); }

void store() {
    Serial.printf("[profile] writing %s to VESC flash\n", RIDER_PROFILES[s_index].name);
    vescLink.storeConfig();
}

} // namespace profiles
