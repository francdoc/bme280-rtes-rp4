# bme280-rtes-rp4

This repository implements a real-time, non-blocking driver for the BME280 sensor on Linux using POSIX message queues and multi-threaded Active Objects.

## Overview

Four threads coordinate via named message queues to sample and report environmental data:

1. **Time Sequencer (TS):**

   * Generates two `POLL_REQ` requests to the Hardware Access Object (HAO) at half-intervals.
   * Sends one `STATUS_REQ` to the Operational Active Object (OPAO) per interval.

2. **Bus Access Object (BAO):**

   * Receives `POLL_REQ` from HAO, performs a single-byte I²C command, reads 8 raw bytes from the BME280 sensor, and replies with `POLL_RES:<count>` + raw payload.

3. **Hardware Access Object (HAO):**

   * Forwards `POLL_REQ` to BAO, decodes `POLL_RES` raw data into calibrated temperature, pressure, and humidity values using the BME280 compensation algorithm, stores the latest readings, and on `STATUS_REQ` replies with `STATUS_RES:<count> T=<°C> P=<Pa> H=<%>`.

4. **Operational Active Object (OPAO):**

   * Forwards `STATUS_REQ` from TS to HAO and logs the formatted `STATUS_RES` to stdout.

Each thread uses a single `poll()` loop over its mqueue file descriptors, ensuring it handles all inputs (messages and raw data) in a unified event-handling loop without inline blocking or cross-thread callbacks.

## Build and Run

```bash
sudo build_run.sh
```

**Notes:**

* Default I²C bus is `1` (`/dev/i2c-1`) and address `0x76`. Change these in `main.c` if needed.
* Sampling interval is defined by `INTERVAL_SEC` (default 1 second) in `ts.c`.

## Logging

* **TS, BAO, HAO, OPAO** print timestamps and message details to stdout or stderr.
* **BAO** and **HAO** output raw hex dumps to stderr for debugging.
* **OPAO** prints processed `STATUS_RES` lines clearly to stdout.
