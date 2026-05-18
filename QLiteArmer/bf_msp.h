/*
  bf_msp.h — Betaflight MSP Protocol Header

  Public interface for bf_msp.cpp.

  Internal DisplayPort helpers (dp_heartbeat, dp_clear, dp_draw, dp_write)
  are static functions inside bf_msp.cpp and must NOT be declared here.
  Declaring them here as extern while defining them static in the .cpp
  causes the "declared extern and later static" compiler error.
*/

#pragma once

#include <Arduino.h>
#include "Telemetry.h"
#include "CrossfireELRS.h"

// -------------------------------------------------------
// VTX System Type
// Exposed so other modules can query which system is connected
// -------------------------------------------------------
typedef enum {
    VTX_UNKNOWN   = 0,
    VTX_DJI_V1    = 1,
    VTX_DJI_O3    = 2,
    VTX_WALKSNAIL = 3
} VtxSystemType;

// -------------------------------------------------------
// Public API
// -------------------------------------------------------

// Call once on startup — pass the serial port used for MSP
// and the Telemetry instance for voltage/altitude data
void bf_msp_init(HardwareSerial& serial, Telemetry& telemetry, CrossfireELRS& elrs);

// Call every loop() on the MSP core
// Reads incoming bytes from the Air Unit and responds to polls
void bf_msp_parse_incoming();

// Call once to trigger the initial config burst
// Used by state_machine.cpp when VTX connection is confirmed
void bf_msp_firstbeat_start();

// Call every loop() — sends initial config burst after first connection
// Non-blocking — steps through frames with small gaps between each
// Returns automatically when complete, then becomes a no-op
void bf_msp_firstbeat_update();

// Call every loop() — sends periodic status/analog/altitude frames
// Also sends DisplayPort heartbeat to Walksnail Avatar when detected
void bf_msp_heartbeat_update(bool armed);

// Call every loop() — sends DisplayPort OSD writes to O3/O4 and Avatar
// Automatically skipped for DJI V1 (uses native MSP display instead)
// Non-blocking — steps through clear/write/draw cycle
void bf_msp_dp_update_osd_nb();

// Call whenever armed state changes
void bf_msp_set_armed(bool armed);

// Returns which VTX system has been detected
// Returns VTX_UNKNOWN until enough MSP polls have been received
VtxSystemType bf_msp_get_vtx_type();