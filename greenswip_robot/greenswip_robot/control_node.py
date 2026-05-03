#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.Kp    = 1.2
        self.v_max = 0.4
        self.v_min = 0.15
        self.d_max = 0.524
        self.dband = 0.05
        self._cx = self._area = self._det_flag = 0.0
        self._last_det_time = self.get_clock().now()

        self.sub = self.create_subscription(Float32MultiArray, '/box_detection', self.cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.loop)
        self.get_logger().info('ControlNode started')

    def cb(self, msg):
        if len(msg.data) < 4:
            return
        self._cx, _, self._area, self._det_flag = msg.data[:4]
        if self._det_flag > 0.5:
            self._last_det_time = self.get_clock().now()

    def loop(self):
        cmd = Twist()

        if self._det_flag < 0.5:
            elapsed = (self.get_clock().now() - self._last_det_time).nanoseconds * 1e-9
            if elapsed > 2.0:
                # Arc search - Ackermann safe (forward + steer, no spin)
                cmd.linear.x = self.v_min
                t = self.get_clock().now().nanoseconds * 1e-9
                cmd.angular.z = 0.3 if int(t/3)%2==0 else -0.3
            else:
                cmd.linear.x = self.v_min
            self.pub.publish(cmd)
            return

        # ARRIVED check - stop briefly then re-enable (no permanent lock)
        if self._area >= 0.25:
            self.get_logger().info(f'ARRIVED! area={self._area:.3f}')
            self.pub.publish(cmd)  # publish stop
            return

        error = self._cx if abs(self._cx) > self.dband else 0.0
        angular = -self.Kp * error
        angular = max(-self.d_max, min(self.d_max, angular))
        speed = self.v_max * (1.0 - 0.6 * abs(angular)/self.d_max)
        speed = max(self.v_min, speed)

        cmd.linear.x = speed
        cmd.angular.z = angular
        self.pub.publish(cmd)
        self.get_logger().info(f'cx={self._cx:+.2f} ang={math.degrees(angular):+.1f} v={speed:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
