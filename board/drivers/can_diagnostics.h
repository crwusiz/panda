#pragma once

#include "board/can_diagnostics.h"

// Only diagnostic state is written here. No logging, allocation, or communication
// takes place in CAN/SPI interrupts. Each snapshot starts a new measurement window.
static uint32_t can_diag_enqueued_us[PANDA_CAN_CNT][CAN_TX_BUFFER_SIZE];
static can_diagnostics_t can_diag;

static uint16_t can_diag_u16(uint32_t value) {
  return (uint16_t)MIN(value, 0xFFFFU);
}

static int can_diag_bus(const can_ring *q) {
  int bus = -1;
  if (q != &can_rx_q) {
    for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
      if (q == can_queues[i]) {
        bus = i;
        break;
      }
    }
  }
  return bus;
}

// Called with the queue's existing critical section held, before advancing w_ptr.
static void can_diag_enqueue(const can_ring *q, uint32_t next_w_ptr) {
  const int bus = can_diag_bus(q);
  if (bus >= 0) {
    can_diag_enqueued_us[bus][q->w_ptr] = microsecond_timer_get();
    const uint32_t queued = (next_w_ptr >= q->r_ptr) ? (next_w_ptr - q->r_ptr) : (q->fifo_size - q->r_ptr + next_w_ptr);
    can_diag.bus[bus].peak = MAX(can_diag.bus[bus].peak, (uint16_t)queued);
  }
}

// Called with the queue's existing critical section held, before advancing r_ptr.
static void can_diag_dequeue(const can_ring *q) {
  const int bus = can_diag_bus(q);
  if (bus >= 0) {
    const uint32_t wait_us = microsecond_timer_get() - can_diag_enqueued_us[bus][q->r_ptr];
    can_diag.bus[bus].wait_max_us = MAX(can_diag.bus[bus].wait_max_us, wait_us);
  }
}

static void can_diag_drop(const can_ring *q) {
  const int bus = can_diag_bus(q);
  if (bus >= 0) {
    can_diag.bus[bus].dropped = can_diag_u16((uint32_t)can_diag.bus[bus].dropped + 1U);
  }
}

// Called from process_can's existing critical section.
static void can_diag_tx_time(uint8_t bus, uint32_t start_us) {
  const uint16_t elapsed_us = can_diag_u16(microsecond_timer_get() - start_us);
  can_diag.bus[bus].tx_max_us = MAX(can_diag.bus[bus].tx_max_us, elapsed_us);
}

static void can_diag_rx_safety_time(uint8_t bus, uint32_t start_us) {
  const uint16_t elapsed_us = can_diag_u16(microsecond_timer_get() - start_us);
  ENTER_CRITICAL();
  can_diag.bus[bus].rx_safety_max_us = MAX(can_diag.bus[bus].rx_safety_max_us, elapsed_us);
  EXIT_CRITICAL();
}

void can_diagnostics_spi_nack(void) {
  ENTER_CRITICAL();
  if (can_diag.spi_nacks == 0U) {
    for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
      can_diag.first_nack_queued[i] = (uint16_t)(CAN_TX_BUFFER_SIZE - 1U - can_slots_empty(can_queues[i]));
    }
  }
  can_diag.spi_nacks += 1U;
  EXIT_CRITICAL();
}

void can_diagnostics_spi_write(uint32_t start_us) {
  const uint16_t elapsed_us = can_diag_u16(microsecond_timer_get() - start_us);
  ENTER_CRITICAL();
  can_diag.spi_write_max_us = MAX(can_diag.spi_write_max_us, elapsed_us);
  EXIT_CRITICAL();
}

static inline void can_diagnostics_snapshot(can_diagnostics_t *out, const uint16_t *tx_fifo_free) {
  ENTER_CRITICAL();
  can_diag.time_us = microsecond_timer_get();
  for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
    const can_ring *q = can_queues[i];
    const uint16_t queued = (uint16_t)(CAN_TX_BUFFER_SIZE - 1U - can_slots_empty(q));
    can_diag.bus[i].queued = queued;
    can_diag.bus[i].peak = MAX(can_diag.bus[i].peak, queued);
    can_diag.bus[i].tx_fifo_free = tx_fifo_free[i];
    if (queued > 0U) {
      const uint32_t oldest_us = can_diag.time_us - can_diag_enqueued_us[i][q->r_ptr];
      can_diag.bus[i].wait_max_us = MAX(can_diag.bus[i].wait_max_us, oldest_us);
    }
  }
  *out = can_diag;
  // Queue timestamps, contents, pointers, and transport flags are never reset here.
  can_diag = (can_diagnostics_t){0};
  EXIT_CRITICAL();
}
