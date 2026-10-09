"""Offline jerk-limited position samples. Nothing here is sent to a drive."""

from __future__ import annotations

import math

from cia402 import R23_COUNTS_PER_REV, degrees_to_counts

CYCLE_S = 0.001


def one_degree_counts(units_per_rev: float = R23_COUNTS_PER_REV) -> int:
    return degrees_to_counts(1.0, units_per_rev)


def _accel_distance(speed: float, accel: float, jerk: float) -> tuple[float, float, float]:
    """Distance, jerk time, and constant-accel time to reach speed."""
    if speed <= 0:
        return 0.0, 0.0, 0.0
    if accel * accel >= speed * jerk:
        jerk_time = math.sqrt(speed / jerk)
        accel_time = 0.0
        used = jerk * jerk_time
    else:
        used = accel
        jerk_time = accel / jerk
        accel_time = speed / accel - jerk_time
    distance = (
        used * jerk_time * jerk_time
        + 1.5 * used * jerk_time * accel_time
        + 0.5 * used * accel_time * accel_time
    )
    return distance, jerk_time, accel_time


def _segments(distance: float, velocity: float, accel: float, jerk: float) -> list[tuple[float, float]]:
    if distance < 0 or velocity <= 0 or accel <= 0 or jerk <= 0:
        raise ValueError("trajectory limits must be positive")
    full, _, _ = _accel_distance(velocity, accel, jerk)
    if 2.0 * full <= distance:
        peak = velocity
        cruise = (distance - 2.0 * full) / velocity
    else:
        low = 0.0
        high = velocity
        for _ in range(60):
            mid = 0.5 * (low + high)
            half, _, _ = _accel_distance(mid, accel, jerk)
            if 2.0 * half > distance:
                high = mid
            else:
                low = mid
        peak = low
        cruise = 0.0
    half, jerk_time, accel_time = _accel_distance(peak, accel, jerk)
    return [
        (jerk_time, jerk),
        (accel_time, 0.0),
        (jerk_time, -jerk),
        (cruise, 0.0),
        (jerk_time, -jerk),
        (accel_time, 0.0),
        (jerk_time, jerk),
    ]


def _position_at(segments: list[tuple[float, float]], time_s: float) -> float:
    position = velocity = acceleration = elapsed = 0.0
    for duration, jerk in segments:
        if time_s <= elapsed + duration or duration == 0:
            dt = max(0.0, time_s - elapsed)
            return (
                position
                + velocity * dt
                + 0.5 * acceleration * dt * dt
                + jerk * dt * dt * dt / 6.0
            )
        position += (
            velocity * duration
            + 0.5 * acceleration * duration * duration
            + jerk * duration * duration * duration / 6.0
        )
        velocity += acceleration * duration + 0.5 * jerk * duration * duration
        acceleration += jerk * duration
        elapsed += duration
    return position


def generate(
    start: int,
    target: int,
    max_velocity: float,
    max_acceleration: float,
    max_jerk: float,
    cycle_s: float = CYCLE_S,
) -> list[int]:
    """Fixed-cycle command positions from start to target, inclusive."""
    if cycle_s <= 0:
        raise ValueError("cycle time must be positive")
    distance = target - start
    if distance == 0:
        return [start]
    sign = 1 if distance > 0 else -1
    segments = _segments(abs(distance), max_velocity, max_acceleration, max_jerk)
    total = sum(duration for duration, _jerk in segments)
    count = max(1, int(math.ceil(total / cycle_s)))
    samples = [start]
    for index in range(1, count):
        position = start + sign * _position_at(segments, index * cycle_s)
        command = int(round(position))
        if sign > 0:
            command = min(target, max(samples[-1], command))
        else:
            command = max(target, min(samples[-1], command))
        samples.append(command)
    samples.append(target)
    if samples[-2] == target:
        samples.pop()
    return samples


def preview_one_degree(
    start: int = 0,
    units_per_rev: float = R23_COUNTS_PER_REV,
    speed_rpm: float = 5.0,
    accel_rpm_per_s: float = 20.0,
    jerk_time_s: float = 0.05,
) -> list[int]:
    """About one shaft degree at the verified R23 scale. Offline only."""
    counts = one_degree_counts(units_per_rev)
    velocity = speed_rpm / 60.0 * units_per_rev
    acceleration = accel_rpm_per_s / 60.0 * units_per_rev
    jerk = acceleration / jerk_time_s
    return generate(start, start + counts, velocity, acceleration, jerk)
