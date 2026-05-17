#!/usr/bin/env python3
"""Apply joystick deadband and scaling for manual robot-control tests."""

from __future__ import annotations

import argparse


def apply_deadband(value: float, deadband: float = 0.05) -> float:
    if abs(value) < abs(deadband):
        return 0.0
    return value


def scale_axis(value: float, limit: float, deadband: float = 0.05) -> float:
    value = max(-1.0, min(1.0, apply_deadband(value, deadband)))
    return value * abs(limit)


def main() -> int:
    parser = argparse.ArgumentParser(description="Scale a normalized joystick axis.")
    parser.add_argument("value", type=float, help="Input axis value in [-1, 1].")
    parser.add_argument("--limit", type=float, default=1.0, help="Output limit.")
    parser.add_argument("--deadband", type=float, default=0.05, help="Deadband around zero.")
    args = parser.parse_args()

    print(scale_axis(args.value, args.limit, args.deadband))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())