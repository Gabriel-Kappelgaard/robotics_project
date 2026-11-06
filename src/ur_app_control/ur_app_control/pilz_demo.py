#!/usr/bin/env python3
"""Round trip: moveJ nudge, moveL up 5 cm, moveL back, moveJ back."""
import rclpy

from ur_app_control.ur_moves import UrMotion


def main(args=None):
    rclpy.init(args=args)
    ur = UrMotion()
    ok = False
    try:
        q_start = ur.current_joints()
        q_nudge = list(q_start)
        q_nudge[0] += 0.2                                   # rad

        ok = ur.move_j(q_nudge)
        if ok:
            ur.wait(0.5)                                    # let the robot settle
            p_low = ur.current_pose()
            p_high = list(p_low)
            p_high[2] += 0.05                               # 5 cm up
            ok = ur.move_l(p_high)
            if ok:
                ur.wait(0.5)
                ok = ur.move_l(p_low)
            if ok:
                ur.wait(0.5)
                ok = ur.move_j(q_start)
        ur.get_logger().info('demo finished' if ok else 'demo stopped on a failed move')
    finally:
        ur.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
