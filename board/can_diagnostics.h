#pragma once

#include <stdint.h>

// Optional, read-only diagnostics. The existing CAN and health formats are unchanged.
#define CAN_DIAGNOSTICS_REQUEST 0xD9U
#define CAN_DIAGNOSTICS_VERSION 1U

typedef struct __attribute__((packed)) {
  uint16_t queued;
  uint16_t peak;
  uint32_t wait_max_us;
  uint16_t tx_max_us;
  uint16_t rx_safety_max_us;
  uint16_t dropped;
  uint16_t tx_fifo_free;
} can_bus_diagnostics_t;

typedef struct __attribute__((packed)) {
  uint32_t time_us;
  uint32_t spi_nacks;
  uint16_t spi_write_max_us;
  uint16_t first_nack_queued[3];
  can_bus_diagnostics_t bus[3];
} can_diagnostics_t;
