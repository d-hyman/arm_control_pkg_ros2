#stands in for YOLO. 
#It cycles through "bottle" (goes left), "apple" (goes right), 
#and "cell phone" (no bin, so it's ignored).

"""
fake_detector_node: stands in for the YOLO package until it works.

Publishes /detections (vision_msgs/Detection2DArray) in exactly the format the
real AI node should use: one object in the centre of the frame for
`hold_time` seconds, then nothing for `gap_time`, cycling through `classes`.
"""
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class FakeDetectorNode(Node):
    def __init__(self):
        super().__init__('fake_detector_node')
        p = self.declare_parameter
        self.classes = list(p('classes', ['bottle', 'apple', 'cell phone']).value)
        self.hold = p('hold_time', 3.0).value
        self.gap = p('gap_time', 2.0).value
        self.w = float(p('image_width', 320).value)
        self.h = float(p('image_height', 240).value)

        self.pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.t0 = self.now()
        self.create_timer(0.1, self.tick)  # ~10 fps, like the ESP32 stream

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        t = self.now() - self.t0
        period = self.hold + self.gap
        cls = self.classes[int(t // period) % len(self.classes)]

        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'

        if (t % period) < self.hold:
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = self.w / 2
            det.bbox.center.position.y = self.h / 2
            det.bbox.size_x = 80.0
            det.bbox.size_y = 80.0
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = cls
            hyp.hypothesis.score = 0.9
            det.results.append(hyp)
            msg.detections.append(det)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDetectorNode()
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