"""Small timeout helper so state waits can be tested without the drive."""

from __future__ import annotations


def wait_until(predicate, timeout_s: float, sleep, now, interval_s: float = 0.02):
    """Call predicate until it is true or the deadline passes.

    `now` returns seconds. `sleep` receives the poll interval. Both are injected
    so tests can advance a fake clock.
    """
    if timeout_s < 0:
        raise ValueError("timeout must be >= 0")
    deadline = now() + timeout_s
    while True:
        if predicate():
            return
        if now() >= deadline:
            raise TimeoutError(f"condition not met within {timeout_s}s")
        remaining = deadline - now()
        sleep(interval_s if remaining > interval_s else remaining)
