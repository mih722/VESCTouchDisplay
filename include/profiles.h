#pragma once
#include <Arduino.h>
#include "config.h"

namespace profiles {

void begin();                       // restore the last used profile from NVS
uint8_t index();
const RiderProfile &current();

// Select a profile: stores it and pushes the limits to the VESC.
void set(uint8_t idx);
void next();

// Re-send the current limits (the VESC forgets them across a power cycle,
// since we deliberately never write them to its flash).
void reapply();

// ...unless the rider explicitly asks, which is what this is: persist the
// VESC's current live configuration - the selected profile's limits included
// - to its flash, so it holds with or without this dashboard. Deliberately
// rare; see PROFILE_LONG_PRESS_WRITE_MS in config.h.
void store();

} // namespace profiles
