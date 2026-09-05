#pragma once
// XVS camera-sync interrupt. Still a stub, but no longer a blocked one: the
// move to the Feather RP2040 frees GP12, and config.h reserves it as XVS_PIN.
// What remains is the handler itself (attachInterrupt on XVS_PIN, plus the
// RING_SETTLE_MS / XVS_TIMEOUT_US sequencing), not the routing.
inline void xvs_init() {}
