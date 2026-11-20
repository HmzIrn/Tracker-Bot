"""Unit tests for GoalPoseManager and free-space nudging logic."""

import math
import os
import sys
import pytest

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from goal_manager import GoalPoseManager, transform_world_to_map, nudge_to_free_space


def test_transform_world_to_map():
    # Translation (2, 3), rotation 90 deg (pi/2)
    # (x_w, y_w) = (1, 0) -> R * (1, 0) = (0, 1) -> + (2, 3) = (2, 4)
    res = transform_world_to_map(1.0, 0.0, [2.0, 3.0, math.pi / 2.0])
    assert abs(res[0] - 2.0) < 1e-5
    assert abs(res[1] - 4.0) < 1e-5

    # Identity transform
    res_ident = transform_world_to_map(5.5, -3.2, [0.0, 0.0, 0.0])
    assert abs(res_ident[0] - 5.5) < 1e-5
    assert abs(res_ident[1] - (-3.2)) < 1e-5


def test_dwell_timing_and_once_per_episode():
    mgr = GoalPoseManager(staying_dwell_sec=5.0, goal_cooldown_sec=15.0)

    # t = 0.0s to 4.9s: should NOT trigger
    for t in [0.0, 1.0, 2.0, 3.0, 4.0, 4.9]:
        goal = mgr.update_track(track_id=1, activity="Staying", world_pos=(1.0, 2.0), sim_time=t)
        assert goal is None

    # t = 5.0s: exactly 5s dwell, should trigger!
    goal = mgr.update_track(track_id=1, activity="Staying", world_pos=(1.0, 2.0), sim_time=5.0)
    assert goal is not None
    assert abs(goal[0] - 1.0) < 1e-3 and abs(goal[1] - 2.0) < 1e-3

    mgr.mark_fired(track_id=1, sim_time=5.0)

    # t = 5.5s, 6.0s, 10.0s: while still staying, should NOT trigger again (once per episode)
    for t in [5.5, 6.0, 8.0, 10.0]:
        goal = mgr.update_track(track_id=1, activity="Staying", world_pos=(1.0, 2.0), sim_time=t)
        assert goal is None


def test_cooldown_between_episodes():
    mgr = GoalPoseManager(staying_dwell_sec=5.0, goal_cooldown_sec=15.0)

    # Track 1 triggers at t=5.0
    for t in [0.0, 2.0, 5.0]:
        goal = mgr.update_track(track_id=1, activity="Staying", world_pos=(0.0, 0.0), sim_time=t)
    assert goal is not None
    mgr.mark_fired(track_id=1, sim_time=5.0)

    # Track 2 starts staying at t=1.0. At t=6.0, its dwell is 5.0s, but cooldown (5.0s + 15.0s = 20.0s) has NOT elapsed.
    mgr.update_track(track_id=2, activity="Staying", world_pos=(3.0, 3.0), sim_time=1.0)
    goal2 = mgr.update_track(track_id=2, activity="Staying", world_pos=(3.0, 3.0), sim_time=6.0)
    assert goal2 is None  # Blocked by cooldown!

    # At t=20.1s (cooldown expired), Track 2 should be allowed to trigger
    goal2_late = mgr.update_track(track_id=2, activity="Staying", world_pos=(3.0, 3.0), sim_time=20.1)
    assert goal2_late is not None
    assert abs(goal2_late[0] - 3.0) < 1e-3


def test_reset_on_leaving_staying():
    mgr = GoalPoseManager(staying_dwell_sec=5.0, goal_cooldown_sec=15.0)

    # Track 1 stays from 0.0 to 4.0s
    for t in [0.0, 2.0, 4.0]:
        mgr.update_track(track_id=1, activity="Staying", world_pos=(0.0, 0.0), sim_time=t)

    # Track 1 walks at 4.5s (leaves Staying before 5s dwell)
    mgr.update_track(track_id=1, activity="Walking", world_pos=(0.5, 0.5), sim_time=4.5)

    # Track 1 resumes staying at 5.0s. At 9.0s (dwell only 4.0s from 5.0s), should NOT fire
    mgr.update_track(track_id=1, activity="Staying", world_pos=(1.0, 1.0), sim_time=5.0)
    goal = mgr.update_track(track_id=1, activity="Staying", world_pos=(1.0, 1.0), sim_time=9.0)
    assert goal is None

    # At 10.0s (5.0s since new episode start at 5.0s), should fire!
    goal = mgr.update_track(track_id=1, activity="Staying", world_pos=(1.0, 1.0), sim_time=10.0)
    assert goal is not None


def test_reset_on_disappearing():
    mgr = GoalPoseManager(staying_dwell_sec=5.0, goal_cooldown_sec=15.0)
    mgr.update_track(track_id=1, activity="Staying", world_pos=(0.0, 0.0), sim_time=0.0)
    mgr.update_track(track_id=1, activity="Staying", world_pos=(0.0, 0.0), sim_time=3.0)

    # Track disappears
    mgr.cleanup(active_track_ids=set())
    assert 1 not in mgr.episodes


def test_median_position_over_dwell():
    mgr = GoalPoseManager(staying_dwell_sec=5.0, goal_cooldown_sec=15.0)

    # Jittering positions: X = [1.0, 1.1, 0.9, 1.0, 5.0(outlier)], Y = [2.0, 2.0, 2.1, 1.9, 2.0]
    positions = [(1.0, 2.0), (1.1, 2.0), (0.9, 2.1), (1.0, 1.9), (5.0, 2.0)]
    times = [0.0, 1.0, 2.0, 3.0, 5.0]

    for p, t in zip(positions, times):
        goal = mgr.update_track(track_id=1, activity="Staying", world_pos=p, sim_time=t)

    assert goal is not None
    # Median of [0.9, 1.0, 1.0, 1.1, 5.0] is 1.0 (outlier 5.0 is rejected)
    assert abs(goal[0] - 1.0) < 1e-3
    assert abs(goal[1] - 2.0) < 1e-3


def test_free_space_nudging():
    # Grid 10x10, resolution 0.1m, origin (0, 0)
    # Cell (5, 5) corresponds to x=0.55m, y=0.55m
    grid = [0] * 100

    # Case 1: Target cell is free -> returned unchanged
    res = nudge_to_free_space(0.55, 0.55, grid, 10, 10, 0.1, 0.0, 0.0, max_radius=0.5)
    assert res == (0.55, 0.55)

    # Case 2: Target cell (5, 5) is occupied (100)
    grid[5 * 10 + 5] = 100
    res = nudge_to_free_space(0.55, 0.55, grid, 10, 10, 0.1, 0.0, 0.0, max_radius=0.5)
    assert res is not None
    # Distance to nudged point should be <= 0.5m
    dist = math.hypot(res[0] - 0.55, res[1] - 0.55)
    assert dist <= 0.5
    assert dist > 0.0

    # Case 3: Entire 0.5m radius is occupied
    all_occupied = [100] * 100
    res_none = nudge_to_free_space(0.55, 0.55, all_occupied, 10, 10, 0.1, 0.0, 0.0, max_radius=0.5)
    assert res_none is None
