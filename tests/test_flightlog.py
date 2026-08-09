# Off-target tests for the per-arm flight record (pure Python, no `machine`
# imports): `python3 tests/test_flightlog.py`.

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware"))

import flightlog


def _ticks_diff(a, b):
    return a - b


def _use_tmp_files(tmp_path):
    flightlog.LOG_FILE = str(tmp_path / "flightlog.json")
    flightlog._TMP_FILE = str(tmp_path / "flightlog.json.tmp")


def test_records_the_shape_the_bench_needs(tmp_path):
    _use_tmp_files(tmp_path)
    rec = flightlog.Recorder(started_ms=1000, free_at_arm=40000)
    for body_us in (3000, 55000, 4000):
        rec.note_iteration(body_us)
    rec.note_overrun()
    rec.note_free(31000)
    rec.note_free(38000)  # higher: must not replace the minimum
    session = rec.record(duration_ms=2000, exit_reason="disarm")

    assert session["iterations"] == 3
    assert session["hz"] == 1  # 3 iterations in 2000 ms
    assert session["overruns"] == 1
    assert session["worst_body_us"] == 55000
    assert session["min_free"] == 31000
    assert session["free_at_arm"] == 40000
    assert session["exit"] == "disarm"


def test_failsafe_outages_are_timed():
    rec = flightlog.Recorder()
    rec.note_failsafe(now_ms=5000, silence_ms=520)
    rec.note_failsafe_cleared(now_ms=5800, ticks_diff=_ticks_diff)
    rec.note_failsafe(now_ms=9000, silence_ms=610)
    rec.note_failsafe_cleared(now_ms=9300, ticks_diff=_ticks_diff)
    session = rec.record(duration_ms=10000, exit_reason="disarm")
    assert session["failsafes"] == 2
    assert session["failsafe_ms"] == [800, 300]
    assert session["max_silence_ms"] == 610


def test_failsafe_still_open_at_disarm_is_recorded():
    # An outage that lasted until the session ended never gets a cleared
    # event, and it is the most interesting kind -- it must not vanish.
    rec = flightlog.Recorder()
    rec.note_failsafe(now_ms=5000, silence_ms=700)
    session = rec.record(duration_ms=6000, exit_reason="link_timeout")
    assert session["failsafes"] == 1
    assert session["failsafe_ms"] == [-1]
    assert session["exit"] == "link_timeout"


def test_failsafe_detail_is_bounded_but_the_count_is_not():
    # A link flapping thousands of times must not grow a list in RAM on a
    # flying aircraft.
    rec = flightlog.Recorder()
    for i in range(flightlog.MAX_FAILSAFE_DETAIL * 5):
        rec.note_failsafe(now_ms=i * 100, silence_ms=500)
        rec.note_failsafe_cleared(now_ms=i * 100 + 50, ticks_diff=_ticks_diff)
    session = rec.record(duration_ms=100000, exit_reason="disarm")
    assert session["failsafes"] == flightlog.MAX_FAILSAFE_DETAIL * 5
    assert len(session["failsafe_ms"]) == flightlog.MAX_FAILSAFE_DETAIL


def test_sessions_persist_and_are_capped(tmp_path):
    _use_tmp_files(tmp_path)
    flightlog.clear()
    for index in range(flightlog.MAX_SESSIONS + 4):
        rec = flightlog.Recorder(free_at_arm=index)
        rec.note_iteration(1000)
        assert flightlog.append(rec.record(1000, "disarm"))
    sessions = flightlog.load()
    assert len(sessions) == flightlog.MAX_SESSIONS
    # Oldest dropped, newest kept and in order.
    assert sessions[-1]["free_at_arm"] == flightlog.MAX_SESSIONS + 3
    assert sessions[0]["free_at_arm"] == 4


def test_load_survives_a_corrupt_file(tmp_path):
    _use_tmp_files(tmp_path)
    with open(flightlog.LOG_FILE, "w") as handle:
        handle.write("{not json")
    assert flightlog.load() == []


def test_append_reports_failure_rather_than_raising(tmp_path):
    # Runs inside the flight loop's finally block, so a failing filesystem
    # must not turn a normal disarm into an exception on top of whatever
    # caused the exit.
    _use_tmp_files(tmp_path)
    flightlog.LOG_FILE = str(tmp_path / "nonexistent-dir" / "flightlog.json")
    flightlog._TMP_FILE = str(tmp_path / "nonexistent-dir" / "tmp.json")
    rec = flightlog.Recorder()
    rec.note_iteration(1000)
    assert flightlog.append(rec.record(1000, "disarm")) is False


if __name__ == "__main__":
    import tempfile
    import pathlib

    failures = 0
    for name, test in sorted(globals().items()):
        if name.startswith("test_") and callable(test):
            try:
                with tempfile.TemporaryDirectory() as tmp:
                    if test.__code__.co_argcount:
                        test(pathlib.Path(tmp))
                    else:
                        test()
                print("ok   " + name)
            except AssertionError as error:
                failures += 1
                print("FAIL " + name + ": " + str(error))
    sys.exit(1 if failures else 0)
