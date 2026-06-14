#!/usr/bin/env python3
import argparse
import sys
import time

from piper_sdk import C_PiperInterface_V2


def stamp(message):
    return getattr(message, "time_stamp", 0.0)


def mode_text(value):
    try:
        return f"0x{int(value):x}"
    except (TypeError, ValueError):
        return str(value)


def main() -> int:
    parser = argparse.ArgumentParser(description="Check whether Piper feedback is fresh on SocketCAN.")
    parser.add_argument("--can-port", default="can0")
    parser.add_argument("--duration", type=float, default=4.0)
    args = parser.parse_args()

    # The SDK's built-in bitrate probe can fail in constrained launch/sandbox
    # environments even when `ip -details link show can0` works. The wrapper
    # checks and resets SocketCAN externally, so keep this probe focused on
    # whether Piper feedback actually arrives.
    piper = C_PiperInterface_V2(can_name=args.can_port, judge_flag=False)
    piper.ConnectPort(piper_init=False)

    deadline = time.time() + max(0.5, args.duration)
    latest = 0.0
    status_summary = ""
    try:
        while time.time() < deadline:
            status = piper.GetArmStatus()
            joints = piper.GetArmJointMsgs()
            low = piper.GetArmLowSpdInfoMsgs()
            high = piper.GetArmHighSpdInfoMsgs()
            gripper = piper.GetArmGripperMsgs()
            latest = max(
                latest,
                stamp(status),
                stamp(joints),
                stamp(low),
                stamp(high),
                stamp(gripper),
            )
            arm_status = status.arm_status
            status_summary = (
                f"isOk={piper.isOk()} latest_ts={latest:.6f} "
                f"enable={piper.GetArmEnableStatus()} "
                f"ctrl_mode={mode_text(arm_status.ctrl_mode)} "
                f"teach={mode_text(arm_status.teach_status)} "
                f"err={arm_status.err_code}"
            )
            if latest > 0.0:
                print(f"OK: Piper feedback received on {args.can_port}: {status_summary}")
                return 0
            time.sleep(0.1)
    finally:
        try:
            piper.DisconnectPort()
        except Exception:
            pass

    print(f"FAIL: no Piper feedback on {args.can_port}: {status_summary}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
