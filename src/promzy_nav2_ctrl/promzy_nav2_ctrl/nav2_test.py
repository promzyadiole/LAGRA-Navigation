#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main():
    # --- Init
    rclpy.init()
    nav = BasicNavigator()

    # --- Set initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # --- Wait for Nav2
    nav.waitUntilNav2Active()

    # --- Send Nav2 goal
    waypoints = []
    waypoints.append(create_pose_stamped(nav, -3.0, 0.0, 3.14)) #First destination still in the starting room (Parlour)
    waypoints.append(create_pose_stamped(nav, -4.0, 3.0, 3.14)) #Kitchen (Second room)
    waypoints.append(create_pose_stamped(nav, 2.0, 2.0, 0.0)) #Just after door of second room
    waypoints.append(create_pose_stamped(nav, 8.0, 3.0, 1.57)) #Dining room (3rd Room)
    waypoints.append(create_pose_stamped(nav, 8.0, -3.0, 1.57)) #Master Bedroom
    waypoints.append(create_pose_stamped(nav, 5.0, -1.5, 0.0)) #Closer to exit door of Master Bedroom
    waypoints.append(create_pose_stamped(nav, 2.5, -1.5, -1.57)) # Aside exit door master Bedroom
    waypoints.append(create_pose_stamped(nav, 3.2, -5.5, -1.57)) # Garage ---Capture what you see here
    waypoints.append(create_pose_stamped(nav, -3.0, -5.5, 3.14)) # Wooden block (store area) ---Capture what you see here
    waypoints.append(create_pose_stamped(nav, -2.0, -5.5, 1.57)) # Closer to exit of store area
    waypoints.append(create_pose_stamped(nav, -2.0, -3.0, 0.0)) #Return back to Parlour (Now in Parlour)
    waypoints.append(create_pose_stamped(nav, -3.0, 0.0, -1.57)) # Parlour ---Capture what you see in this part of the parlour
    # --- Go to one pose
    # nav.goToPose(goal_pose1)
    # while not nav.isTaskComplete():
    #     feedback = nav.getFeedback()
    #     # print(feedback)

    # --- Follow waypoints
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # print(feedback)

    print(nav.getResult())

    # --- Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()