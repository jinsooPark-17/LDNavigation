import os
import sys
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
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

waypoints = [Pose( Point(28.0, 9.50, 0), Quaternion(0,0,0,1) ),\
            Pose( Point(28.0, 18.0, 0), Quaternion(0,0,1,0) ),\
            Pose( Point(0.00, 18.0, 0), Quaternion(0,0,-0.707,0.707) ),]

class TestEnv:
    def __init__(self):
        self.entity = "marvin"
        self.move_base = SimpleActionClient('{}/move_base'.format(self.entity), MoveBaseAction)
        self.move_base.wait_for_server()
        self.bag = rosbag.Bag('record-{}.bag'.format(sys.argv[1]), 'w')
        self.record = False

        self.sub_odom = rospy.Subscriber("/{}/odom".format(self.entity),
                                         Odometry,
                                         self.get_odom)
        self.sub_amcl = rospy.Subscriber("/{}/amcl_pose".format(self.entity),
                                         PoseWithCovarianceStamped,
                                         self.get_amcl)
        self.sub_lidar = rospy.Subscriber("/{}/scan".format(self.entity),
                                          LaserScan,
                                          self.get_lidar_scan)

        # localize robot to remove uncertainty
        relocalize = rospy.Publisher("/{}/initialpose".format(self.entity), 
                                     PoseWithCovarianceStamped, 
                                     queue_size=1)
        init_pose = PoseWithCovarianceStamped(  )
        init_pose.header.frame_id = "{}/level_mux_map".format(self.entity)
        init_pose.header.stamp = rospy.Time.now()
        init_pose.pose = rospy.wait_for_message('/{}/odom'.format(self.entity),
                                                Odometry,
                                                timeout=10).pose
        for i in range(10):
            relocalize.publish( init_pose )
            rospy.sleep(0.1)

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
        goal.target_pose.header.frame_id = "{}/level_mux_map".format(self.entity)
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        self.move_base.send_goal_and_wait( goal )

    def test(self):
        random.shuffle( waypoints )
        for waypoint in waypoints:
            print("Moving to ({:2.2f},{:2.2f})".format(waypoint.position.x, waypoint.position.y) )
            self.move( waypoint )
        self.record = False

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python record_random_waypoints.py $NUM_TRIAL")
        sys.exit(1)
    
    rospy.init_node('test_no_gui_move')
    env = TestEnv()
    env.test()
    env.bag.close()

    # Draw trajectory and corresponding lidar scan from bag file to image
    bag = rosbag.Bag('record-{}.bag'.format(sys.argv[1]))
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
    plt.savefig('result-{}.png'.format(sys.argv[1]))
