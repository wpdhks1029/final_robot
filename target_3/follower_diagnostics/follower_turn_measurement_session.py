"""Explicit separate entry point for fixed +3/+3 RPM, five-second ground turn."""
from follower_motor_session import main

if __name__=='__main__':raise SystemExit(main(measurement=True,turn=True))
