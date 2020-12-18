import os
import random
import rospy
import rosbag

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
# Draw tools
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

waypoint = [Pose( Point(28.0, 9.50, 0), Quaternion(0,0,0,1) ),\
            Pose( Point(28.0, 18.0, 0), Quaternion(0,0,1,0) ),\
            Pose( Point(0.00, 18.0, 0), Quaternion(0,0,-0.707,0.707) ),]

class TestEnv:
    def __init__(self):
        self.move_base = SimpleActionClient('marvin/move_base', MoveBaseAction)
        self.move_base.wait_for_server()

        self.bag = rosbag.Bag('tmp.bag', 'w')
        self.record = False

        self.sub_odom = rospy.Subscriber("/marvin/odom",
                                         Odometry,
                                         self.get_odom)
        self.sub_amcl = rospy.Subscriber("/marvin/amcl_pose",
                                         PoseWithCovarianceStamped,
                                         self.get_amcl)
        self.sub_lidar = rospy.Subscriber("/marvin/scan",
                                          LaserScan,
                                          self.get_lidar_scan)
        rospy.sleep(5) # wait for messages
        self.record = True

    def get_odom(self, odom):
        self.odom = odom.pose.pose # Pose
    def get_amcl(self, amcl):
        self.amcl = amcl.pose.pose
    def get_lidar_scan(self, lidar):
        self.scan = lidar
        if self.record == True:
            self.bag.write('odom', self.odom)
            self.bag.write('amcl', self.amcl)
            self.bag.write('scan', self.scan)

    def move(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "marvin/level_mux_map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        self.move_base.send_goal_and_wait( goal )

    def test(self):
        for waypoint in random.shuffle( waypoints ):
            self.move( waypoint )

if __name__ == "__main__":
    rospy.init_node('test_no_gui_move')
    env = TestEnv()
    env.test()
    env.bag.close()

    # Draw trajectory and corresponding lidar scan from bag file to image
    bag = rosbag.Bag('tmp.bag')
    odom, amcl, xs, ys = [],[],[],[]
    i=1
    for topic, msg, t in bag.read_messages(topics=bag.get_type_and_topic_info()[1].keys()):
        if i%3==0:
            a_min = msg.angle_min
            a_max = msg.angle_max
            da = msg.angle_increment

            theta = np.arange(a_min, a_max+da, da)[::-1] + odom[-1][2]
            xs.append( odom[-1][0] + msg.ranges * np.cos(theta) )
            ys.append( odom[-1][1] + msg.ranges * np.sin(theta) )
        elif i%3==1:
            (roll, pitch, yaw) = euler_from_quaternion([msg.orientation.x,
                                                        msg.orientation.y,
                                                        msg.orientation.z,
                                                        msg.orientation.w])
            odom.append([msg.position.x,msg.position.y, yaw])
        elif i%3==2:
            (roll, pitch, yaw) = euler_from_quaternion([msg.orientation.x,
                                                        msg.orientation.y,
                                                        msg.orientation.z,
                                                        msg.orientation.w])
            amcl.append([msg.position.x,msg.position.y, yaw])
        i = (i+1)%3
    odom = np.array(odom)
    amcl = np.array(amcl)
    xs = np.array(xs)
    ys = np.array(ys)

    plt.scatter(xs,ys, s=1, c='k')
    plt.plot(odom[:,0], odom[:,1], c='b', label = 'odom')
    plt.plot(amcl[:,0], amcl[:,1], c='r', linestyle=":", label = "amcl")
    plt.legend()
    plt.savefig('test_result.png')
    # remove bag file
    os.remove('tmp.bag')
