#!/usr/bin/env python3
"""Reference MAVLink 2 frames for tests/test_mavlink_scenario.c, encoded by pymavlink (independent of the model).

  python3 tests/data/gen_mavlink_frames.py   # prints C arrays; pymavlink 2.4.41 was used
"""
from pymavlink.dialects.v20 import common as mav

def frame(msg, seq, sysid=255, compid=190):
    m = mav.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    m.seq = seq
    return msg.pack(m)

cases = [
    ("hb_seq0", frame(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, mav.MAV_STATE_ACTIVE, 3), 0)),
    ("sp_seq1", frame(mav.MAVLink_set_position_target_local_ned_message(
        100, 1, 1, mav.MAV_FRAME_LOCAL_NED, 0x0DF8, 1.5, -2.25, -1.0, 0, 0, 0, 0, 0, 0, 0.5, 0), 1)),
    ("sp_zero_seq7", frame(mav.MAVLink_set_position_target_local_ned_message(
        0, 1, 1, mav.MAV_FRAME_LOCAL_NED, 0x0DF8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), 7)),
    ("arm_seq2", frame(mav.MAVLink_command_long_message(1, 1, mav.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0), 2)),
    ("mode_seq3", frame(mav.MAVLink_command_long_message(1, 1, mav.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0), 3)),
    ("hb_sys1_seq255", frame(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, mav.MAV_STATE_ACTIVE, 3), 255, 1, 2)),
]
for name, b in cases:
    print(f"static const uint8 {name}[] = {{ /* {len(b)} bytes */")
    for i in range(0, len(b), 16):
        print("\t" + ", ".join(f"0x{x:02X}" for x in b[i:i+16]) + ",")
    print("};")
