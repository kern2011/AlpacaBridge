### Fixed
- **Sky-Watcher: a quiet serial board can no longer wedge the whole driver** (AlpacaCore):
  on a USB CDC-ACM port that does not honour `VMIN`/`VTIME` as a read timeout (the
  EQ-AL55i Pro's STM32 virtual COM port), a bare `read()` in the mis-paired-reply
  settle and in the reply read loop parked forever in `n_tty_read` when the board went
  quiet, holding the protocol-wrapper I/O mutex and the driver mutex above it so every
  Alpaca request blocked until the service was killed (diagnosed on the rig with gdb:
  one worker stuck in `settle_serial` -> `read`, every other worker blocked on the
  driver mutex; `/proc/<tid>/syscall` = read, `wchan` = `n_tty_read`). The Sky-Watcher
  serial reads are now bounded by `poll()` and run on a non-blocking fd, so a read can
  never park regardless of the tty's `VMIN`/`VTIME` (or a spurious `poll()` readable):
  a quiet board now times out and retransmits as intended instead of hanging. Other
  serial vendors are unchanged.
- **Sky-Watcher: a byte-dropped (malformed) serial reply is resent, not failed** (AlpacaCore):
  slew-time electrical noise on the EQ-AL55i Pro's CDC-ACM link occasionally corrupts a
  reply (a dropped byte, e.g. a truncated `:j1` "25278"); the wrapper now settles and
  resends once -- the same recovery it already used for a mis-paired reply -- instead of
  throwing and failing the whole operation. A `!` board rejection is still never resent.

### Added (tests)
- **Sky-Watcher serial: a read that ignores its timeout cannot wedge the link settle**
  (AlpacaCore): models a tty that returns data at once when present but otherwise blocks
  instead of timing out, and asserts the poll-bounded settle/exchange stays within the
  command timeout.
