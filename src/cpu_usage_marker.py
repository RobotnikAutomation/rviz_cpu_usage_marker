#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from system_monitor.msg import Diagnostic


class CpuMarker:
    def __init__(
            self,
            node_name='cpu_usage_marker',
            frame_id='rb1_base_map',
            topic_to_subscribe='/system_monitor/diagnostics',
            publish_topic_name='cpu_usage_marker',
    ):
        rospy.init_node(node_name)
        try:
            self._frame_id = rospy.get_param(
                '~frame_id',
                frame_id
            )
        except NameError:
            self._frame_id = frame_id

        try:
            self._topic_to_subscribe = rospy.get_param(
                '~topic_to_subscribe',
                topic_to_subscribe
            )
        except NameError:
            self._topic_to_subscribe = topic_to_subscribe

        try:
            self._publish_topic_name = rospy.get_param(
                '~publish_topic_name',
                publish_topic_name
            )
        except NameError:
            self._publish_topic_name = publish_topic_name

        try:
            self._debug = rospy.get_param(
                '~debug',
                debug
            )
            print(str(self._debug))
        except NameError:
            self._debug = False

        self._cpu_usage = rospy.Subscriber(
            name=self._topic_to_subscribe,
            data_class=Diagnostic,
            callback=self.show_cpu_usage
        )
        self._marker_publisher = rospy.Publisher(
            name=self._publish_topic_name,
            data_class=Marker,
            queue_size=10
        )
        self._marker = Marker(
            type=Marker.TEXT_VIEW_FACING,
            id=0,
            lifetime=rospy.Duration(1.5),
            pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
            scale=Vector3(1, 1, 1),
            header=Header(frame_id=self._frame_id),
            color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
            text=""
        )

    def spin(self):
        rospy.spin()

    def show_cpu_usage(self, msg):
        current_cpu_usage = msg.diagCpuUsage.status.load_now
        current_cpu_usage = round(current_cpu_usage, 1)
        current_cpu_usage = str(current_cpu_usage)
        current_cpu_usage += "%"
        if self._debug:
            rospy.loginfo(
                rospy.get_caller_id() + "CPU USAGE: %s", current_cpu_usage
            )
        self.show_text_in_rviz(current_cpu_usage)

    def show_text_in_rviz(self, text):
        self._marker.text = text
        self._marker_publisher.publish(self._marker)


def main():
    cpu_marker = CpuMarker()
    cpu_marker.spin()

if __name__ == '__main__':
    main()
