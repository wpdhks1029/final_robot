"""Explicit separate entry point for the fixed 3RPM/5-second measurement session."""
from follower_motor_session import main

if __name__=='__main__':raise SystemExit(main(measurement=True))
