import tf_transformations
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


class Nav2Controller:
    def __init__(self):
        self.navigator = BasicNavigator()

    def _create_pose(self, x, y, yaw):
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def initialize(self, x=0.0, y=0.0, yaw=0.0):
        initial_pose = self._create_pose(x, y, yaw)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

    def follow_waypoints(self, waypoints):
        poses = [self._create_pose(*wp) for wp in waypoints]
        self.navigator.followWaypoints(poses)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()

        return self.navigator.getResult()

    def go_to_pose(self, x, y, yaw):
        pose = self._create_pose(x, y, yaw)
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()

        return self.navigator.getResult()
