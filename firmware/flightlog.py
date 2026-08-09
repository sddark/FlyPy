# Per-arm flight record, written to flash on disarm and read back later.
#
# Exists because serial capture is the wrong instrument for the failures
# worth catching. Chasing a motor that cut out at part throttle, the USB
# device disconnected mid-run -- the link dies under exactly the conditions
# being investigated, so the evidence disappears at the moment it matters.
# Recording to RAM while armed and writing once on disarm means the test can
# be run with nothing attached at all, and read afterwards with the battery
# out.
#
# Nothing here is written while the aircraft is armed. The flight loop only
# updates counters in a preallocated object -- no allocation, no file I/O,
# no flash latency on the control path. The single write happens after the
# outputs are already safe.
#
# Pure Python (no `machine` imports) so it is testable off-target.

import os

LOG_FILE = "flightlog.json"
_TMP_FILE = "flightlog.json.tmp"

# Sessions kept, newest last. Ten covers a bench session's worth of arm
# cycles without letting the file grow unbounded on a 2 MB filesystem.
MAX_SESSIONS = 10
# Per-session failsafe outages recorded individually. Beyond this only the
# count keeps rising: a link dropping more than twenty times in one arm has
# already made its point, and the list must not grow without bound in RAM.
MAX_FAILSAFE_DETAIL = 20


class Recorder:
    # One instance per armed session. Deliberately preallocates every field
    # in __init__ so that note_*() during flight only ever rebinds existing
    # attributes or appends to a bounded list.
    def __init__(self, started_ms=0, free_at_arm=0):
        self.started_ms = started_ms
        self.free_at_arm = free_at_arm
        self.iterations = 0
        self.overruns = 0
        self.worst_body_us = 0
        self.failsafes = 0
        self.failsafe_ms = []
        self.max_silence_ms = 0
        self.min_free = free_at_arm
        self.exit_reason = "unknown"
        self._lost_at_ms = None

    def note_iteration(self, body_us):
        self.iterations += 1
        if body_us > self.worst_body_us:
            self.worst_body_us = body_us

    def note_overrun(self):
        self.overruns += 1

    def note_silence(self, silence_ms):
        if silence_ms > self.max_silence_ms:
            self.max_silence_ms = silence_ms

    def note_free(self, free_bytes):
        if free_bytes < self.min_free:
            self.min_free = free_bytes

    def note_failsafe(self, now_ms, silence_ms):
        self.failsafes += 1
        self._lost_at_ms = now_ms
        self.note_silence(silence_ms)

    def note_failsafe_cleared(self, now_ms, ticks_diff):
        # ticks_diff is injected so this module stays free of `time`, whose
        # wrapping helpers differ between MicroPython and CPython.
        if self._lost_at_ms is None:
            return
        if len(self.failsafe_ms) < MAX_FAILSAFE_DETAIL:
            self.failsafe_ms.append(ticks_diff(now_ms, self._lost_at_ms))
        self._lost_at_ms = None

    def record(self, duration_ms, exit_reason):
        self.exit_reason = exit_reason
        # A failsafe still open at disarm never got a cleared event, so its
        # outage would otherwise vanish from the record entirely -- and an
        # outage that lasted until disarm is the most interesting kind.
        if self._lost_at_ms is not None and len(self.failsafe_ms) < MAX_FAILSAFE_DETAIL:
            self.failsafe_ms.append(-1)  # -1: still lost when the session ended
        hz = 0
        if duration_ms > 0:
            hz = self.iterations * 1000 // duration_ms
        return {
            "duration_ms": duration_ms,
            "iterations": self.iterations,
            "hz": hz,
            "overruns": self.overruns,
            "worst_body_us": self.worst_body_us,
            "failsafes": self.failsafes,
            "failsafe_ms": self.failsafe_ms,
            "max_silence_ms": self.max_silence_ms,
            "free_at_arm": self.free_at_arm,
            "min_free": self.min_free,
            "exit": self.exit_reason,
        }


def load():
    try:
        import json

        with open(LOG_FILE) as handle:
            stored = json.load(handle)
        if isinstance(stored, dict) and isinstance(stored.get("sessions"), list):
            return stored["sessions"]
    except (OSError, ValueError):
        pass
    return []


def append(session):
    # Best-effort by contract, not by accident: this runs in the flight
    # loop's finally block, so a full or failing filesystem must not turn a
    # normal disarm into an exception on top of whatever caused the exit.
    # Returns True when the record actually reached flash.
    try:
        import json

        sessions = load()
        sessions.append(session)
        if len(sessions) > MAX_SESSIONS:
            sessions = sessions[-MAX_SESSIONS:]
        with open(_TMP_FILE, "w") as handle:
            json.dump({"sessions": sessions}, handle)
        os.rename(_TMP_FILE, LOG_FILE)
        return True
    except (OSError, ValueError, MemoryError):
        return False


def clear():
    for path in (LOG_FILE, _TMP_FILE):
        try:
            os.remove(path)
        except OSError:
            pass
