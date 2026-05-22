"""Unit tests for MorphState — pure Python, no ROS2 runtime needed."""
import pytest

from airsim_ros2_bridge.formation_node import MorphState


def test_progress_zero_at_start():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    assert ms.progress(100.0) == 0.0


def test_progress_half_at_midpoint():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    assert ms.progress(100.75) == pytest.approx(0.5, rel=1e-3)


def test_progress_one_at_end():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    assert ms.progress(101.5) == 1.0


def test_progress_clamped_above_one():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    assert ms.progress(200.0) == 1.0


def test_progress_clamped_below_zero():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    assert ms.progress(99.0) == 0.0


def test_pause_freezes_progress():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    ms.pause(100.75)              # paused at midpoint
    assert ms.progress(101.5) == pytest.approx(0.5, rel=1e-3)
    assert ms.progress(105.0) == pytest.approx(0.5, rel=1e-3)


def test_resume_continues_from_pause_point():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    ms.pause(100.75)              # paused at midpoint
    ms.resume(103.0)              # 2.25s gap
    # After resume, elapsed = (now - t0) - paused_elapsed = (now - 100) - 2.25
    # At now=103.75: elapsed = 3.75 - 2.25 = 1.5 → progress = 1.0
    assert ms.progress(103.75) == 1.0
    # At now=103.0 (just resumed): elapsed = 3.0 - 2.25 = 0.75 → progress = 0.5
    assert ms.progress(103.0) == pytest.approx(0.5, rel=1e-3)


def test_double_pause_is_idempotent():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    ms.pause(100.75)
    ms.pause(101.0)               # 2nd pause should be no-op
    ms.resume(103.0)              # gap should be 103.0 - 100.75 = 2.25, not 2.0
    assert ms.progress(103.75) == 1.0


def test_resume_without_pause_is_noop():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    ms.resume(101.0)              # no-op
    assert ms.progress(101.5) == 1.0    # progress as if no pause/resume happened


def test_chained_pause_resume():
    """pause → resume → pause → resume 가 누적 paused_elapsed 로 정확히 합산되는지."""
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    ms.pause(100.75)        # freeze at 0.5
    ms.resume(103.0)        # paused_elapsed = 2.25, back to running from 0.5
    ms.pause(103.25)        # progress = (3.25 - 2.25)/1.5 = 0.666..., freeze here
    assert ms.progress(110.0) == pytest.approx(2.0 / 3.0, rel=1e-3)
    ms.resume(115.0)        # paused_elapsed += 11.75 → total 14.0
    assert ms.progress(115.5) == 1.0   # (15.5 - 14.0)/1.5 = 1.0


def test_duration_zero_raises_value_error():
    """duration<=0 은 잘못된 구성 → ValueError (silent runtime crash 방지)."""
    with pytest.raises(ValueError, match='duration must be > 0'):
        MorphState(src_pattern='A', dst_pattern='B', t0_sec=0.0, duration=0.0)


def test_duration_negative_raises_value_error():
    with pytest.raises(ValueError, match='duration must be > 0'):
        MorphState(src_pattern='A', dst_pattern='B', t0_sec=0.0, duration=-1.0)
