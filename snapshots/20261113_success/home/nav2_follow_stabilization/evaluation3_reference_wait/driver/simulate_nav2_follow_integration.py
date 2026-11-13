#!/usr/bin/env python3
"""Deterministic protocol simulation for Nav2 + continuous follower integration.

This exercises the real fail-closed command-gate core with synthetic Nav2,
follower, motor and odometry samples. It does not claim to simulate Nav2's
planner, LiDAR physics, motor dynamics, or collision geometry.
"""
import json
import math
import random

from nav2_convoy_gate_core import Nav2ConvoyGateCore


def feed(core, now, *, phase='FOLLOW', follow_reason='FORWARD_ALLOWED',
         follow_v=.02, gap=1.5, command=(.15, .08), motor_ok=True,
         graph_ok=True):
    core.graph_ok = graph_ok
    core.update_nav(*command, now)
    core.update_follower(dict(trial_phase=phase, reason='CONTINUOUS_READY',
                              follower_follow_reason=follow_reason,
                              follower_requested_v=follow_v), now)
    core.update_motor(dict(armed=motor_ok, fault=None, io_failed=False,
                           feedback_fresh=True), now)
    core.update_leader_pose(gap, 0., now)
    core.update_follower_pose(0., 0., now)


def timeline_simulation():
    core = Nav2ConvoyGateCore()
    dt = .05
    leader_x, follower_x = 1.5, 0.
    opened = held = stale_stops = 0
    min_gap = math.inf
    max_v = 0.
    phase_counts = {'NAV2_STARTUP': 0, 'FOLLOW': 0, 'PLANNING': 0,
                    'AVOID': 0, 'FOLLOW_AFTER_REJOIN': 0}
    for tick in range(1600):
        now = 100. + tick * dt
        t = tick * dt
        if t < 2.:
            phase, follow_reason, fv = 'FOLLOW', 'STOP_LEADER_STOPPED', 0.
            label = 'NAV2_STARTUP'
        elif t < 20.:
            phase, follow_reason, fv = 'FOLLOW', 'FORWARD_ALLOWED', .02
            label = 'FOLLOW'
        elif t < 24.:
            phase, follow_reason, fv = 'PLANNING', 'STOP_LEADER_STOPPED', 0.
            label = 'PLANNING'
        elif t < 34.:
            phase, follow_reason, fv = 'AVOID', 'STOP_LEADER_STOPPED', 0.
            label = 'AVOID'
        else:
            phase, follow_reason, fv = 'FOLLOW', 'FORWARD_ALLOWED', .02
            label = 'FOLLOW_AFTER_REJOIN'
        phase_counts[label] += 1
        gap = leader_x - follower_x
        # Deliberately inject a stale status and a stale Nav2 command. Both
        # should cause a zero output before the simulated source recovers.
        stale_status = 12. <= t < 12.5
        stale_nav = 40. <= t < 40.5
        feed(core, now - (.50 if stale_status else 0.), phase=phase,
             follow_reason=follow_reason, follow_v=fv, gap=gap,
             command=(.15, .08), motor_ok=True, graph_ok=True)
        if stale_nav:
            core.update_nav(.15, .08, now - .40)
        out = core.evaluate(now)
        min_gap = min(min_gap, gap)
        max_v = max(max_v, abs(out.linear_x))
        if out.reason == 'NAV2_FOLLOWER_GATE_OPEN':
            opened += 1
        else:
            held += 1
            if out.reason in ('STOP_FOLLOWER_STATUS_STALE', 'STOP_NAV2_COMMAND_STALE'):
                stale_stops += 1
        # Simple 1-D plant: the follower tracks at the recorded requested
        # velocity; during obstacle planning/avoidance it may advance around
        # the box while the leader is held by the gate.
        leader_x += out.linear_x * dt
        follower_speed = fv if phase in ('FOLLOW', 'FOLLOW_AFTER_REJOIN') else (.01 if phase == 'AVOID' else 0.)
        follower_x += min(.025, max(0., follower_speed)) * dt
        if gap <= .85 and out.linear_x > 0:
            raise AssertionError('leader advanced inside minimum convoy gap')
        if phase in ('PLANNING', 'AVOID') and (out.linear_x != 0 or out.angular_z != 0):
            raise AssertionError('leader command passed while follower was planning/avoiding')
        if abs(out.linear_x) > .04 + 1e-12 or abs(out.angular_z) > .10 + 1e-12:
            raise AssertionError('hard command limit exceeded')
    if opened == 0 or held == 0 or stale_stops == 0:
        raise AssertionError('expected open, hold, and stale-input scenarios')
    return dict(ticks=1600, simulated_seconds=1600 * dt,
                gate_open_ticks=opened, gate_hold_ticks=held,
                stale_input_stop_ticks=stale_stops,
                minimum_gap_m=round(min_gap, 4), maximum_linear_mps=round(max_v, 4),
                phase_ticks=phase_counts, invariant_pass=True)


def randomized_fault_simulation(seed=20261107, trials=20000):
    rng = random.Random(seed)
    core = Nav2ConvoyGateCore()
    counts = {}
    for i in range(trials):
        now = 10. + i * .01
        phase = rng.choice(('FOLLOW', 'PLANNING', 'AVOID', 'STOP'))
        gap = rng.uniform(.3, 3.)
        command = (rng.uniform(-.2, .2), rng.uniform(-.3, .3))
        motor_ok = rng.random() > .05
        graph_ok = rng.random() > .02
        status_age = rng.choice((0., .05, .2, .36, .6))
        nav_age = rng.choice((0., .04, .2, .26, .5))
        follow_reason = rng.choice(('FORWARD_ALLOWED', 'STOP_FOLLOW_DISTANCE',
                                    'STOP_LEADER_STOPPED', 'STOP_NEAR_LEADER'))
        feed(core, now, phase=phase, follow_reason=follow_reason,
             follow_v=rng.uniform(0., .025), gap=gap, command=command,
             motor_ok=motor_ok, graph_ok=graph_ok)
        core.follower = (core.follower[0], now - status_age)
        core.nav = (*core.nav[:2], now - nav_age)
        out = core.evaluate(now)
        counts[out.reason] = counts.get(out.reason, 0) + 1
        if abs(out.linear_x) > .04 + 1e-12 or abs(out.angular_z) > .10 + 1e-12:
            raise AssertionError('randomized hard limit violation')
        if phase != 'FOLLOW' and (out.linear_x != 0 or out.angular_z != 0):
            raise AssertionError('randomized non-FOLLOW phase was allowed')
        if not graph_ok and (out.linear_x != 0 or out.angular_z != 0):
            raise AssertionError('randomized graph ambiguity was allowed')
        if not motor_ok and (out.linear_x != 0 or out.angular_z != 0):
            raise AssertionError('randomized motor fault was allowed')
        if gap <= .85 and (out.linear_x != 0 or out.angular_z != 0):
            raise AssertionError('randomized unsafe gap was allowed')
    return dict(seed=seed, trials=trials, reasons=counts, invariant_pass=True)


def speed_sweep():
    rows = []
    startup = Nav2ConvoyGateCore()
    feed(startup, 1., follow_reason='STOP_LEADER_STOPPED', follow_v=0.,
         command=(.15, 0.), gap=1.5)
    startup_output = startup.evaluate(1.05)
    if startup_output.linear_x <= .03 or startup_output.linear_x > .04:
        raise AssertionError('startup command must release the existing >0.03 m/s follower start threshold')
    for cap in (.015, .020, .025, .030, .040):
        core = Nav2ConvoyGateCore(max_linear=cap)
        feed(core, 1., follow_v=.02, command=(.15, .04), gap=1.5)
        out = core.evaluate(1.05)
        rows.append(dict(configured_cap_mps=cap, allowed_mps=round(out.linear_x, 4),
                         gate_reason=out.reason))
    return dict(follower_tracking=rows,
                startup=dict(allowed_mps=startup_output.linear_x,
                             existing_follower_start_threshold_mps=.03,
                             gate_reason=startup_output.reason))


def follow_only_simulation():
    """Check pursuit-only output and that a follower stop holds Nav2."""
    core = Nav2ConvoyGateCore()
    feed(core, 1., phase='FOLLOW', follow_reason='FORWARD_ALLOWED',
         follow_v=.02, gap=1.5, command=(.15, .08))
    tracking = core.evaluate(1.05)
    if tracking.reason != 'NAV2_FOLLOWER_GATE_OPEN' or tracking.linear_x > .023 + 1e-12:
        raise AssertionError('pursuit-only leader speed was not bounded to follower demand')
    if not math.isclose(tracking.angular_z * .15, tracking.linear_x * .08,
                        rel_tol=1e-12, abs_tol=1e-12):
        raise AssertionError('gate changed Nav2 curvature')
    feed(core, 2., phase='FOLLOW', follow_reason='STOP_NEAR_LEADER',
         follow_v=0., gap=1.5, command=(.15, .08))
    obstacle_stop = core.evaluate(2.05)
    if obstacle_stop.linear_x != 0. or obstacle_stop.angular_z != 0.:
        raise AssertionError('follower obstacle stop did not hold Nav2')
    return dict(follower_tracking_reason=tracking.reason,
                leader_speed_mps=tracking.linear_x,
                follower_stop_reason=obstacle_stop.reason,
                leader_zero_on_follower_stop=(obstacle_stop.linear_x == 0.
                                              and obstacle_stop.angular_z == 0.),
                local_avoidance_planning=False)


def main():
    print(json.dumps(dict(kind='GATE_PROTOCOL_SIMULATION_ONLY',
                          follow_only=follow_only_simulation(),
                          timeline=timeline_simulation(),
                          randomized=randomized_fault_simulation(),
                          speed_sweep=speed_sweep()), indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
