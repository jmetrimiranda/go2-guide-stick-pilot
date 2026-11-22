"""FAKE unitree_sdk2py for the offline tests of go2_turn_pilot.py. Pure Python, no network.

It is only importable when test/fake_sdk is put on PYTHONPATH on purpose. It never touches a
robot: the "robot" is a thread in this process (see _fake_robot.py, configured by FAKE_* env vars).
"""
import sys

print("FAKE unitree_sdk2py (test stub)", file=sys.stderr, flush=True)
