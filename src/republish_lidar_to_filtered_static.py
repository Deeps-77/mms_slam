#!/usr/bin/env python3
import rospy
import struct
import ast
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

def pack_rgb(r, g, b):
    return struct.unpack('f', struct.pack('I', (int(r) << 16) | (int(g) << 8) | int(b)))[0]

def parse_rgb_param(val):
    def clip3(t):
        return tuple(max(0, min(255, int(x))) for x in t)
    if isinstance(val, (list, tuple)) and len(val) == 3:
        return clip3(val)
    if isinstance(val, str):
        s = val.strip()
        try:
            obj = ast.literal_eval(s)
            if isinstance(obj, (list, tuple)) and len(obj) == 3:
                return clip3(obj)
        except Exception:
            pass
        parts = [p.strip() for p in s.split(',')]
        if len(parts) == 3:
            return clip3(parts)
    rospy.logwarn("rgb param invalid (%r); using default [255,255,255]", val)
    return (255, 255, 255)

class Republisher:
    def __init__(self):
        self.input_topic = rospy.get_param('~input', '/camera/depth/color/points')
        self.output_topic = rospy.get_param('~output', '/filtered_points_static')
        self.out_frame = rospy.get_param('~frame_id', '')  # empty = keep input
        self.rgb = parse_rgb_param(rospy.get_param('~rgb', [255, 255, 255]))
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(self.input_topic, PointCloud2, self.cb, queue_size=1)

    def cb(self, msg):
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1),
        ]
        rgb_f = pack_rgb(*self.rgb)
        pts_list = []
        for x, y, z in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            pts_list.append((float(x), float(y), float(z), rgb_f))
        if not pts_list:
            rospy.logwarn_throttle(5.0, "No valid XYZ points in %s", self.input_topic)
        out = pc2.create_cloud(msg.header, fields, pts_list)
        if self.out_frame:
            out.header.frame_id = self.out_frame
        self.pub.publish(out)

def main():
    rospy.init_node('republish_lidar_to_filtered_static', anonymous=False)
    Republisher()
    rospy.spin()

if __name__ == '__main__':
    main()