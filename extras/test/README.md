# Host-based RS485 tests

These tests compile the RS485 code paths with a host C++ compiler against a
minimal Arduino/AVR stub (`stub/`) and exercise the bus state machine, the
protocol parser, and the overflow/recovery behavior byte by byte.

```sh
make test
```

Covered:

- The DCS-BIOS protocol parser commits the `0x55555555` sync sequence as
  `'U'` characters after a mid-stream gap (bug reproduction), and
  `ProtocolParser::reset()` prevents it.
- An RS485 slave processes broadcast chunks of any size up to the master's
  cap, keeps its bus framing and keeps answering polls during a buffer
  overflow, and recovers from overload by cleanly skipping torn frames.
- Poll answers (empty and with a queued input message) are framed correctly.
- The SYNC quiet-period detection works across 16-bit `micros()` boundaries.
- The RS485 master caps broadcast chunks at `DCSBIOS_RS485_MAX_CHUNK_LENGTH`
  and saturates its PC-side buffer cleanly instead of overwriting unsent data.
