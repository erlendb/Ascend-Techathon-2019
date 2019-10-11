"""
A simple drone library for use in techathon.

Classes:
    - Simdrone: gives control of the drone
    - Camera: give access to camera

See documentation for examples of how to use it.
"""

import rospy
import tf.transformations
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

class SimDrone(object):
    def __init__(self):
        # Communication variables
        self._last_pose = None
        self._last_twist = None
        self._setpoint_msg = PositionTarget()
        self._last_state = State()

        self._camera = Camera(
            imgtopic="drone/camera_front/image",
            infotopic="drone/camera_front/info"
        )

        # ROS Communication
        self._setpoint_pub = rospy.Publisher("mavros/setpoint_raw/local",
                                             PositionTarget, queue_size=1)
        self._pose_sub = rospy.Subscriber("mavros/local_position/pose",
                                          PoseStamped, self._pose_cb)
        self._twist_sub = rospy.Subscriber("mavros/local_position/velocity",
                                           TwistStamped, self._twist_cb)

        self._state_sub = rospy.Subscriber('mavros/state',
                                           State, self._state_cb)

        self._publish_setpoint_active = False
        rospy.Timer(rospy.Duration(0.05), self._publish_setpoint)
        rospy.loginfo("Drone Initialized")

    # -------------------
    # ------ <API> ------
    # -------------------
    def activate(self):
        """
        Arm the drone and place into offboard.
        """
        arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        offboard = rospy.ServiceProxy('mavros/set_mode', SetMode)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            arming(True)
            offboard(0, "OFFBOARD")
            recieved_data = self._last_pose != None and self._last_twist != None
            if recieved_data and self._last_state.mode == "OFFBOARD" and self._last_state.armed:
                self._setpoint_msg.position.x = self._last_pose.pose.position.x
                self._setpoint_msg.position.y = self._last_pose.pose.position.y
                self._setpoint_msg.position.z = self._last_pose.pose.position.z
                break
            rate.sleep()

        self._publish_setpoint_active = True

    def deactivate(self):
        """
        Disarm the drone.
        """
        arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        offboard = rospy.ServiceProxy('mavros/set_mode', SetMode)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            arming(False)
            offboard(0, "MANUAL")
            if self._last_state.mode == "MANUAL" and not self._last_state.armed:
                break

        self._publish_setpoint_active = False

    def takeoff(self, relative_height=5.0):
        """
        Takeoff at current position. Block untill takeoff is completed.
        """
        if not self._publish_setpoint_active:
            self.activate()

        self._setpoint_msg.position.x = self._last_pose.pose.position.x
        self._setpoint_msg.position.y = self._last_pose.pose.position.y
        self._setpoint_msg.position.z = self._last_pose.pose.position.z + relative_height

        rospy.loginfo("Start takeoff")
        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            if abs(self._setpoint_msg.position.z - self._last_pose.pose.position.z) < 1.0:
                break

            rate.sleep()
        rospy.loginfo("Takeoff complete")

    def land(self):
        """
        Attempt to land at current position.
        """
        self._setpoint_msg.position.x = self._last_pose.pose.position.x
        self._setpoint_msg.position.y = self._last_pose.pose.position.y
        self._setpoint_msg.position.z = -10.0
        rospy.loginfo("Starting to land")

        rate = rospy.Rate(1)
        rate.sleep() # give the setpoint time to take effect
        while not rospy.is_shutdown():
            if abs(self._last_twist.twist.linear.z) < 0.05:
                break
            rate.sleep()

        rospy.loginfo("Landing complete")
        self._setpoint_msg.position.x = self._last_pose.pose.position.x
        self._setpoint_msg.position.y = self._last_pose.pose.position.y
        self._setpoint_msg.position.z = self._last_pose.pose.position.z

    def set_target(self, x, y, z=None, yaw=0):
        """
        Set the position and yaw target.
        """
        self._setpoint_msg.position.x = x
        self._setpoint_msg.position.y = y
        if z != None:
            self._setpoint_msg.position.z = z
        self._setpoint_msg.yaw = yaw

        rospy.logdebug("Setpoint: ({},{},{}), {}".format(
            self._setpoint_msg.position.x,
            self._setpoint_msg.position.y,
            self._setpoint_msg.position.z,
            self._setpoint_msg.yaw))

    def get_pose(self):
        """
        Return pose as geometry_msgs/Pose
        """
        return self._last_pose.pose

    @property
    def target(self):
        """
        Return position target of drone.
        """
        return self._setpoint_msg.position

    @property
    def position(self):
        """
        Returns position of drone.
        """
        return self._last_pose.pose.position

    @property
    def velocity(self):
        """
        Return velocity of drone.
        """
        return self._last_twist.twist.linear

    @property
    def yaw(self):
        """
        Return yaw of drone
        """
        return self._get_yaw()

    @property
    def camera(self):
        """
        Give access to the front facing camera of the drone.
        """
        return self._camera

    # -------------------
    # ----- </API> ------
    # -------------------

    # -----------------------
    # ----- <Internal> ------
    # -----------------------
    def _pose_cb(self, posestamped):
        """
        Receive and store PoseStamped msgs from MAVROS.
        """
        self._last_pose = posestamped

    def _twist_cb(self, twiststamped):
        """
        Receive and store PoseStamped msgs from MAVROS.
        """
        self._last_twist = twiststamped

    def _state_cb(self, state):
        """
        Receive and store State msgs from MAVROS.
        """
        self._last_state = state

    def _publish_setpoint(self, _):
        """
        Stamp and publish current setpoint to MAVROS.
        """
        if self._publish_setpoint_active:
            self._setpoint_msg.header.stamp = rospy.Time.now()
            self._setpoint_msg.header.frame_id = "map"
            self._setpoint_pub.publish(self._setpoint_msg)

    def _get_yaw(self):
        q = self._last_pose.pose.orientation
        quat = (q.x, q.y, q.z, q.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        yaw = euler[2]
        return yaw
    # ------------------------
    # ----- </Internal> ------
    # ------------------------

class Camera(object):
    def __init__(self, imgtopic, infotopic):
        self._cv_bridge = CvBridge()
        self._camera_image = Image()
        self._camera_info = CameraInfo()

        self._camera_image_sub = rospy.Subscriber(imgtopic,
                                                  Image,
                                                  self._camera_image_cb)
        self._camera_info_sub = rospy.Subscriber(infotopic,
                                                 CameraInfo,
                                                 self._camera_info_cb)

    @property
    def info(self):
        """
        Return info about the camera used in simulator.
        https://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        """
        return self._camera_info

    @property
    def image(self):
        """
        Return the last image from the camera feed in gazebo.
        """
        return self._camera_image

    # -----------------------
    # ----- <Internal> ------
    # -----------------------
    def _camera_image_cb(self, image_msg):
        """
        Receive the camera feed from gazebo. For internal use only.
        """
        self._camera_image = self._cv_bridge.imgmsg_to_cv2(image_msg)

    def _camera_info_cb(self, info_msg):
        """
        Receive info about camera from gazebo. For internal use only.
        """
        self._camera_info = info_msg
    # ------------------------
    # ----- </Internal> ------
    # ------------------------
