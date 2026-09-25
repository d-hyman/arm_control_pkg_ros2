#sorter_node is the brain. 
#It waits until the same class has been seen confidently for 5 frames in a row near the center of the frame, which filters out YOLO flicker. 
#Then it looks up whether that class goes left or right and runs a pick-and-place sequence: above pick → lower → close gripper → lift → move to bin → release → home. 
#It interpolates smoothly between poses at 20 Hz, ignores detections while it's moving, and waits a cooldown afterward so it doesn't react to stale frames. 
#It reuses the same rel_x/rel_y math as your prototype.

"""
sorter_node: the "brain" of the arm.

Subscribes:  /detections   (vision_msgs/Detection2DArray)  <- from the AI package
             /manual_sort  (std_msgs/String, "left" or "right") <- for testing
Publishes:   /joint_commands (sensor_msgs/JointState, radians, 20 Hz)
             /sorter_status  (std_msgs/String)

Flow: wait for the same object class to be seen confidently for N frames in
the pick zone -> look up which bin it goes in -> run a pick-and-place
sequence of named poses, smoothly interpolated -> return home -> cooldown.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

DEFAULT_JOINTS = [
    'base_waist_joint', 'waist_link1_joint', 'link1-2',
    'link2_3', 'gripper_arm_joint', 'gear_gripper_link',
]
POSE_NAMES = ('home', 'above_pick', 'pick', 'left_drop', 'right_drop')


class SorterNode(Node):
    def __init__(self):
        super().__init__('sorter_node')
        p = self.declare_parameter

        self.joint_names = list(p('joint_names', DEFAULT_JOINTS).value)
        self.conf_thresh = p('confidence_threshold', 0.5).value
        self.frames_to_confirm = p('frames_to_confirm', 5).value
        self.img_w = float(p('image_width', 320).value)
        self.img_h = float(p('image_height', 240).value)
        self.center_tol = p('center_tolerance', 0.5).value
        self.cooldown = p('cooldown', 2.0).value
        self.left_classes = set(p('left_classes', ['bottle']).value)
        self.right_classes = set(p('right_classes', ['apple']).value)
        self.grip_open = p('gripper_open', 0.3).value
        self.grip_closed = p('gripper_closed', 1.2).value
        self.move_dur = p('move_duration', 1.2).value
        self.grip_dur = p('grip_duration', 0.6).value

        n_arm = len(self.joint_names) - 1  # last joint is the gripper
        self.poses = {}
        for name in POSE_NAMES:
            pose = list(p(f'pose_{name}', [1.57] * n_arm).value)
            if len(pose) != n_arm:
                raise ValueError(f'pose_{name} needs {n_arm} values, got {len(pose)}')
            self.poses[name] = pose

        # Motion state
        self.current = self.poses['home'] + [self.grip_open]
        self.steps = []
        self.busy = False
        self.seg_start = list(self.current)
        self.seg_target = list(self.current)
        self.seg_t0 = self.now()
        self.seg_dur = 1.0
        self.cooldown_until = 0.0

        # Detection-confirmation state
        self.candidate = None
        self.count = 0

        self.cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.status_pub = self.create_publisher(String, 'sorter_status', 10)
        self.create_subscription(Detection2DArray, 'detections', self.on_detections, 10)
        self.create_subscription(String, 'manual_sort', self.on_manual, 10)
        self.create_timer(0.05, self.tick)  # 20 Hz

        self.get_logger().info(
            f'Ready. LEFT={sorted(self.left_classes)} RIGHT={sorted(self.right_classes)}')

    # ---------- helpers ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def side_for(self, cls):
        if cls in self.left_classes:
            return 'left'
        if cls in self.right_classes:
            return 'right'
        return None

    # ---------- inputs ----------
    def on_manual(self, msg):
        side = msg.data.strip().lower()
        if side not in ('left', 'right'):
            self.get_logger().warn('manual_sort expects "left" or "right"')
            return
        if self.busy:
            self.get_logger().warn('Busy, ignoring manual command')
            return
        self.start_sequence(self.build_sequence(side), f'manual->{side}')

    def on_detections(self, msg):
        if self.busy or self.now() < self.cooldown_until:
            return

        best_cls, best_score = None, 0.0
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            if hyp.score < self.conf_thresh:
                continue
            # Same relative coords as your prototype: 0 = centre, +-1 = edges
            rel_x = (det.bbox.center.position.x - self.img_w / 2) / (self.img_w / 2)
            rel_y = (det.bbox.center.position.y - self.img_h / 2) / (self.img_h / 2)
            if abs(rel_x) > self.center_tol or abs(rel_y) > self.center_tol:
                continue  # not in the pick zone
            if hyp.score > best_score:
                best_cls, best_score = hyp.class_id, hyp.score

        if best_cls is None:
            self.candidate, self.count = None, 0
            return

        if best_cls == self.candidate:
            self.count += 1
        else:
            self.candidate, self.count = best_cls, 1
        if self.count < self.frames_to_confirm:
            return

        self.candidate, self.count = None, 0
        side = self.side_for(best_cls)
        if side is None:
            self.get_logger().info(f"'{best_cls}' has no bin assigned, ignoring",
                                   throttle_duration_sec=5.0)
            self.cooldown_until = self.now() + self.cooldown
            return

        self.get_logger().info(f'Confirmed {best_cls} ({best_score:.2f}) -> {side}')
        self.start_sequence(self.build_sequence(side), f'{best_cls}->{side}')

    # ---------- motion ----------
    def build_sequence(self, side):
        P, o, c = self.poses, self.grip_open, self.grip_closed
        m, g = self.move_dur, self.grip_dur
        drop = P[f'{side}_drop']
        return [
            ('above pick',    P['above_pick'] + [o], m),
            ('lower',         P['pick'] + [o], m),
            ('close gripper', P['pick'] + [c], g),
            ('lift',          P['above_pick'] + [c], m),
            (f'move {side}',  drop + [c], m * 1.5),
            ('release',       drop + [o], g),
            ('home',          P['home'] + [o], m * 1.5),
        ]

    def start_sequence(self, steps, label):
        self.get_logger().info(f'Starting sequence: {label}')
        self.steps = list(steps)
        self.busy = True
        self.next_step()

    def next_step(self):
        if not self.steps:
            self.busy = False
            self.cooldown_until = self.now() + self.cooldown
            self.publish_status('idle')
            self.get_logger().info('Sequence done')
            return
        label, target, dur = self.steps.pop(0)
        self.seg_start = list(self.current)
        self.seg_target = list(target)
        self.seg_t0 = self.now()
        self.seg_dur = dur
        self.publish_status(label)

    def tick(self):
        if self.busy:
            t = (self.now() - self.seg_t0) / self.seg_dur if self.seg_dur > 0 else 1.0
            t = min(max(t, 0.0), 1.0)
            s = t * t * (3 - 2 * t)  # smoothstep: gentle start/stop for servos
            self.current = [a + (b - a) * s for a, b in zip(self.seg_start, self.seg_target)]
            if t >= 1.0:
                self.next_step()

        # Always publish, so the driver/RViz always know where the arm should be
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(x) for x in self.current]
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SorterNode()
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