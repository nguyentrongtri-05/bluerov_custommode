#!/usr/bin/env python3
"""
Gia lap Ping Altimeter huong toi (RNGFND1, TYPE=10 MAVLink, ORIENT=0) trong SITL.

Script dat mot buc tuong ao vuong goc voi huong mui tau, cach vi tri hien tai
--wall met. Khoang cach gui di = khoang cach theo tia phia truoc mui tau toi tuong,
duoc tinh tu LOCAL_POSITION_NED va ATTITUDE cua SITL.

Vi du:
    python3 fake_wall_ping.py --wall 5
    python3 fake_wall_ping.py --wall 0.3          # dat tuong sat mui -> test tu lui
    python3 fake_wall_ping.py --wall 5 --noise 0.05

AP_FLAKE8_CLEAN
"""

import argparse
import math
import random
import time

from pymavlink import mavutil

parser = argparse.ArgumentParser(description="Virtual wall forward rangefinder for ArduSub SITL")
parser.add_argument("--connect", default="tcp:127.0.0.1:5762", help="SITL SERIAL1 port")
parser.add_argument("--wall", type=float, default=5.0, help="distance from current position to wall (m)")
parser.add_argument("--noise", type=float, default=0.0, help="gaussian noise std dev (m)")
parser.add_argument("--min", type=float, default=0.2, help="sensor min range (m)")
parser.add_argument("--max", type=float, default=30.0, help="sensor max range (m)")
parser.add_argument("--rate", type=float, default=10.0, help="send rate (Hz)")
args = parser.parse_args()

master = mavutil.mavlink_connection(args.connect)
print("Waiting for heartbeat on %s ..." % args.connect)
master.wait_heartbeat()
print("Connected to system %u" % master.target_system)


def request_interval(msg_id, hz):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)


request_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20)
request_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)

pos_n = pos_e = yaw = None
while pos_n is None or yaw is None:
    m = master.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=True, timeout=5)
    if m is None:
        print("No LOCAL_POSITION_NED/ATTITUDE yet (EKF origin set?) ...")
        continue
    if m.get_type() == "LOCAL_POSITION_NED":
        pos_n, pos_e = m.x, m.y
    else:
        yaw = m.yaw

# wall: a line through point P, normal h = initial heading
h_n, h_e = math.cos(yaw), math.sin(yaw)
wall_n = pos_n + args.wall * h_n
wall_e = pos_e + args.wall * h_e
print("Wall placed %.2f m ahead, heading %.1f deg" % (args.wall, math.degrees(yaw)))

t0 = time.time()
period = 1.0 / args.rate
next_send = 0.0
next_print = 0.0
while True:
    m = master.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=True, timeout=period)
    if m is not None:
        if m.get_type() == "LOCAL_POSITION_NED":
            pos_n, pos_e = m.x, m.y
        else:
            yaw = m.yaw

    now = time.time()
    if now < next_send:
        continue
    next_send = now + period

    # perpendicular gap from vehicle to wall, and incidence of the forward beam
    gap = (wall_n - pos_n) * h_n + (wall_e - pos_e) * h_e
    cos_inc = math.cos(yaw) * h_n + math.sin(yaw) * h_e
    if gap <= 0:
        dist = args.min                 # vehicle is at/through the wall
    elif cos_inc > 0.2:
        dist = gap / cos_inc            # beam hits the wall
    else:
        dist = args.max + 1.0           # beam misses the wall -> no echo
    if args.noise > 0:
        dist += random.gauss(0.0, args.noise)
    dist_cm = int(max(0.0, min(dist * 100.0, 65535)))

    master.mav.distance_sensor_send(
        int((now - t0) * 1000) & 0xFFFFFFFF,
        int(args.min * 100), int(args.max * 100), dist_cm,
        mavutil.mavlink.MAV_DISTANCE_SENSOR_ULTRASOUND,
        1,                                          # id
        mavutil.mavlink.MAV_SENSOR_ROTATION_NONE,   # forward, must match RNGFND1_ORIENT=0
        0)

    if now >= next_print:
        next_print = now + 0.5
        print("gap=%6.2f m  sent=%6.2f m" % (gap, dist_cm * 0.01))
