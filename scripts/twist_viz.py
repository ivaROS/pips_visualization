#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, TwistStamped, Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
from tf import transformations
import numpy as np


def getPose(state):
    return Pose(position=Point(x=state[0], y=state[1]),
                orientation=Quaternion(*transformations.quaternion_from_euler(0, 0, state[2])))

class TwistVisualizer():
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'base_link')
        self.sim_time = rospy.get_param('~sim_time', 1.0)
        self.num_steps = rospy.get_param('~num_steps', 100)

        self.viz_pub = rospy.Publisher("path_out", Path, queue_size=1, latch=True)
        self.twist_sub = rospy.Subscriber("twist", Twist, self.twistCB)
        self.twist_stamped_sub = rospy.Subscriber("twist_stamped", TwistStamped, self.twistStampedCB)


    def twistCB(self, data):
        twist_stamped = TwistStamped(header=Header(frame_id=self.frame_id, stamp=rospy.Time.now()), twist=data)
        self.twistStampedCB(twist_stamped)

    def twistStampedCB(self, data):
        path = Path(header=data.header)
        path.poses = [state for state in self.generateStates(data)]
        self.viz_pub.publish(path)

    def generateStates(self, twist_stamped):
        dt = self.sim_time/float(self.num_steps)

        v = twist_stamped.twist.linear.x
        w = twist_stamped.twist.angular.z

        state=np.array([0,0,0],dtype=np.float64)
        for a in range(self.num_steps):
            state+=np.array([v*np.cos(state[2])*dt, v*np.sin(state[2])*dt,w*dt],dtype=state.dtype)
            pose= getPose(state)
            yield PoseStamped(header=twist_stamped.header, pose=pose)


if __name__ == '__main__':
    try:
        rospy.init_node('twist_visualizer')

        TwistVisualizer()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("exception")
