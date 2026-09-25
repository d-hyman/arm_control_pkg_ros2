#is the only node that knows about hardware. 
#In sim mode it just republishes the commands as /joint_states, so the arm moves in RViz. 
#In serial mode it also converts radians to servo degrees, with a per-joint offset, direction, and clamp. 
#It then sends a line like S,90,90,90,90,90,17 to a microcontroller. 
#Your URDF limits are all 0–3.14 rad, which maps neatly onto 0–180° hobby servos.

"""
servo_driver_node: the only node that knows about hardware.

Subscribes: /joint_commands (sensor_msgs/JointState, radians)
Publishes:  /joint_states   (sensor_msgs/JointState) -> robot_state_publisher -> RViz

mode = "sim":    just echoes commands to /joint_states (arm moves in RViz only)
mode = "serial": also sends one line per change to a microcontroller:
                 "S,<deg0>,<deg1>,...,<deg5>\n"   (integer servo degrees)

Hobby servos have no position feedback, so /joint_states is the commanded
position in both modes.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

DEFAULT_JOINTS = [
    'base_waist_joint', 'waist_link1_joint', 'link1-2',
    'link2_3', 'gripper_arm_joint', 'gear_gripper_link',
]


class ServoDriverNode(Node):
    def __init__(self):
        super().__init__('servo_driver_node')
        p = self.declare_parameter

        self.mode = p('mode', 'sim').value
        port = p('serial_port', '/dev/ttyUSB0').value
        baud = p('baud_rate', 115200).value
        self.joint_names = list(p('joint_names', DEFAULT_JOINTS).value)
        n = len(self.joint_names)
        self.offsets = list(p('offsets_deg', [0.0] * n).value)
        self.directions = list(p('directions', [1] * n).value)
        self.min_deg = list(p('min_deg', [0.0] * n).value)
        self.max_deg = list(p('max_deg', [180.0] * n).value)

        self.serial = None
        self.last_sent = None
        if self.mode == 'serial':
            try:
                import serial
                # Note: opening the port resets most Arduino boards (~2 s).
                # Commands are republished at 20 Hz so nothing is lost for long.
                self.serial = serial.Serial(port, baud, timeout=0.1)
                self.get_logger().info(f'Serial open on {port} @ {baud}')
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'Could not open {port}: {e}. Falling back to sim.')
                self.mode = 'sim'

        self.state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(JointState, 'joint_commands', self.on_command, 10)
        self.get_logger().info(f'Servo driver running in "{self.mode}" mode')

    def on_command(self, msg):
        positions = dict(zip(msg.name, msg.position))
        missing = [j for j in self.joint_names if j not in positions]
        if missing:
            self.get_logger().warn(f'Command missing joints: {missing}',
                                   throttle_duration_sec=5.0)
            return
        rads = [positions[j] for j in self.joint_names]

        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = self.joint_names
        state.position = rads
        self.state_pub.publish(state)

        if self.serial is not None:
            self.write_servos(rads)

    def rad_to_servo_deg(self, i, rad):
        # URDF limits are 0..3.14 rad, which maps onto a 0..180 deg servo.
        # direction -1 with offset 180 mirrors a servo mounted the other way.
        deg = self.offsets[i] + self.directions[i] * math.degrees(rad)
        return max(self.min_deg[i], min(self.max_deg[i], deg))

    def write_servos(self, rads):
        degs = [round(self.rad_to_servo_deg(i, r)) for i, r in enumerate(rads)]
        if degs == self.last_sent:
            return
        line = 'S,' + ','.join(str(d) for d in degs) + '\n'
        try:
            self.serial.write(line.encode('ascii'))
            self.last_sent = degs
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Serial write failed: {e}', throttle_duration_sec=2.0)

    def destroy_node(self):
        if self.serial is not None:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()