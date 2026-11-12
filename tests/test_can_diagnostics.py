"""Compile the production CAN queues and diagnostic collector against a fake clock."""
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest

PANDA = Path(__file__).resolve().parents[1]

PROGRAM = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "board/utils.h"
typedef uint32_t GPIO_TypeDef;
static uint32_t now_us;
static unsigned int depth;
#define ENTER_CRITICAL() do { depth++; } while (0)
#define EXIT_CRITICAL() do { assert(depth > 0); depth--; } while (0)
uint32_t microsecond_timer_get(void) { return now_us; }
void print(const char *text) { (void)text; }
void puth(unsigned int value) { (void)value; }
#include "board/drivers/drivers.h"
void process_can(uint8_t bus) { (void)bus; }
bool can_init(uint8_t bus) { (void)bus; return true; }
void refresh_can_tx_slots_available(void) { }
void mpu_safety_open(void) { }
void mpu_safety_close(void) { }
bool safety_tx_hook(CANPacket_t *packet) { (void)packet; return true; }
#include "board/drivers/can_common.h"
#include "board/comms_definitions.h"
#define USBPACKET_MAX_SIZE 64U
#define FDCAN_TXFQS_TFFL 0x3FU
typedef struct { uint32_t TXFQS; } TestCan;
static TestCan test_cans[3];
static TestCan *cans[3] = {&test_cans[0], &test_cans[1], &test_cans[2]};
static unsigned int diagnostic_control(ControlPacket_t *req, uint8_t *resp) {
  unsigned int resp_len = 0;
  switch (req->request) {
    DIAGNOSTIC_CONTROL_CASE
    default: break;
  }
  return resp_len;
}

static const uint16_t hw_free[3] = {4, 8, 16};
static can_diagnostics_t snapshot(void) {
  can_diagnostics_t out;
  uint32_t read[3], write[3];
  for (int i = 0; i < 3; i++) { read[i] = can_queues[i]->r_ptr; write[i] = can_queues[i]->w_ptr; }
  can_diagnostics_snapshot(&out, hw_free);
  assert(depth == 0);
  for (int i = 0; i < 3; i++) { assert(read[i] == can_queues[i]->r_ptr); assert(write[i] == can_queues[i]->w_ptr); }
  assert(tx_buffer_overflow == 0 && rx_buffer_overflow == 0);
  return out;
}

int main(void) {
  _Static_assert(sizeof(can_diagnostics_t) == 64, "diagnostics wire format");
  _Static_assert(sizeof(can_bus_diagnostics_t) == 16, "bus wire format");
  CANPacket_t packet = {0}, popped;
  can_diagnostics_t out;

  // A poll sees an occupied queue's age even when nothing has been transmitted.
  now_us = 100;
  packet.addr = 0x123;
  assert(can_push(can_queues[2], &packet));
  now_us = 50100;
  out = snapshot();
  assert(out.time_us == now_us && out.bus[2].queued == 1 && out.bus[2].peak == 1);
  assert(out.bus[2].wait_max_us == 50000 && out.bus[2].tx_fifo_free == 16);
  now_us = 60100;
  assert(can_pop(can_queues[2], &popped) && popped.addr == packet.addr);
  out = snapshot();
  assert(out.bus[2].queued == 0 && out.bus[2].wait_max_us == 60000);
  assert(snapshot().bus[2].wait_max_us == 0);

  // Filling, dropping, polling and draining every bus preserves FIFO order.
  for (int bus = 0; bus < 3; bus++) {
    can_clear(can_queues[bus]);
    for (int i = 0; i < 415; i++) {
      packet.addr = (uint32_t)i;
      now_us++;
      assert(can_push(can_queues[bus], &packet));
    }
    assert(!can_push(can_queues[bus], &packet));
    out = snapshot();
    assert(out.bus[bus].queued == 415 && out.bus[bus].peak == 415 && out.bus[bus].dropped == 1);
    assert(snapshot().bus[bus].dropped == 0);
    for (int i = 0; i < 415; i++) {
      now_us++;
      assert(can_pop(can_queues[bus], &popped) && popped.addr == (uint32_t)i);
    }
    assert(!can_pop(can_queues[bus], &popped));
    (void)snapshot();
  }

  // Slot and 32-bit microsecond timer wrap do not corrupt the age measurement.
  can_clear(can_queues[0]);
  can_queues[0]->r_ptr = can_queues[0]->w_ptr = 415;
  now_us = UINT32_MAX - 10;
  packet.addr = 0x456;
  assert(can_push(can_queues[0], &packet));
  assert(can_queues[0]->w_ptr == 0);
  now_us = 19;
  assert(can_pop(can_queues[0], &popped) && popped.addr == 0x456);
  assert(snapshot().bus[0].wait_max_us == 30);

  // The first NACK captures all buses once, subsequent NACKs only count up.
  for (int bus = 0; bus < 3; bus++) {
    for (int i = 0; i <= bus; i++) assert(can_push(can_queues[bus], &packet));
  }
  can_diagnostics_spi_nack();
  assert(can_push(can_queues[2], &packet));
  can_diagnostics_spi_nack();
  out = snapshot();
  assert(out.spi_nacks == 2 && out.first_nack_queued[0] == 1);
  assert(out.first_nack_queued[1] == 2 && out.first_nack_queued[2] == 3);
  assert(out.bus[2].queued == 4);
  can_diagnostics_spi_nack();
  out = snapshot();
  assert(out.spi_nacks == 1 && out.first_nack_queued[2] == 4);
  assert(snapshot().spi_nacks == 0);
  for (int i = 0; i < 3; i++) can_clear(can_queues[i]);

  // Timing maxima saturate rather than wrap, and each poll resets only metrics.
  now_us = 100000;
  can_diag_tx_time(0, now_us - 12);
  can_diag_rx_safety_time(2, now_us - 34);
  can_diagnostics_spi_write(now_us - 56);
  out = snapshot();
  assert(out.bus[0].tx_max_us == 12 && out.bus[2].rx_safety_max_us == 34 && out.spi_write_max_us == 56);
  can_diag_rx_safety_time(2, now_us - 70000);
  assert(snapshot().bus[2].rx_safety_max_us == 65535);
  out = snapshot();
  assert(out.bus[0].tx_max_us == 0 && out.bus[2].rx_safety_max_us == 0 && out.spi_write_max_us == 0);

  // Generic RX queue traffic never contributes to TX measurements.
  assert(can_push(&can_rx_q, &packet));
  now_us += 100000;
  assert(can_pop(&can_rx_q, &popped));
  out = snapshot();
  for (int i = 0; i < 3; i++) assert(out.bus[i].peak == 0 && out.bus[i].wait_max_us == 0);

  // Only a complete, supported diagnostic read consumes the measurement window.
  can_diagnostics_spi_nack();
  ControlPacket_t request = {.request = CAN_DIAGNOSTICS_REQUEST, .param1 = 2, .length = 64};
  uint8_t response[64];
  memset(response, 0xA5, sizeof(response));
  assert(diagnostic_control(&request, response) == 0 && can_diag.spi_nacks == 1);
  request.param1 = CAN_DIAGNOSTICS_VERSION;
  request.length = 63;
  assert(diagnostic_control(&request, response) == 0 && can_diag.spi_nacks == 1);
  assert(response[0] == 0xA5);
  request.length = 64;
  test_cans[2].TXFQS = 7;
  assert(diagnostic_control(&request, response) == 64 && can_diag.spi_nacks == 0);
  memcpy(&out, response, sizeof(out));
  assert(out.spi_nacks == 1 && out.bus[2].tx_fifo_free == 7 && depth == 0);
  puts("CAN diagnostics: queue isolation, FIFO, overflow, reset, timer wrap, NACK snapshot and timings passed");
  return 0;
}
"""


class TestCanDiagnostics(unittest.TestCase):
  def test_queue_instrumentation(self):
    compiler = shlex.split(os.environ.get("CC", "cc"), posix=os.name != "nt")
    if not shutil.which(compiler[0]):
      self.skipTest("C compiler unavailable")
    include = os.environ.get("OPENDBC_INCLUDE_PATH")
    if include is None:
      try:
        import opendbc
        include = opendbc.INCLUDE_PATH
      except ImportError:
        include = PANDA.parent / "opendbc_repo"
    include = Path(include)
    with tempfile.TemporaryDirectory() as directory:
      source = Path(directory) / "diagnostics.c"
      executable = Path(directory) / ("diagnostics.exe" if os.name == "nt" else "diagnostics")
      control = (PANDA / "board/main_comms.h").read_text(encoding="utf-8")
      case = control[control.index("    case CAN_DIAGNOSTICS_REQUEST:"):control.index("    // **** 0xd0:")]
      source.write_text(PROGRAM.replace("DIAGNOSTIC_CONTROL_CASE", case), encoding="utf-8")
      build = subprocess.run([*compiler, "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function", "-O0",
                              "-fsanitize=undefined", "-fno-sanitize-recover=undefined", "-I", str(PANDA), "-I", str(include),
                              str(source), "-o", str(executable)], capture_output=True, text=True)
      self.assertEqual(build.returncode, 0, build.stdout + build.stderr)
      result = subprocess.run([str(executable)], check=True, capture_output=True, text=True, timeout=15)
      self.assertIn("passed", result.stdout)


if __name__ == "__main__":
  unittest.main()
