#!/usr/bin/env python

import rospy
import time
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty


x = 0
y = 0
z = 0
yaw = 0


def poseCallback(pose_message):
    """pose callback, update global variable when receive msg

    :param pose_message: turtlesim pose msg
    :type pose_message: ros msg
    """
    global x
    global y, z, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta
    # print("pose callback")
    # print('x = {}'.format(x))
    # print('y = {}'.format(y))
    # print('yaw = {}'.format(yaw))


def move(speed, distance, is_forward=True):
    """move turtlebot
    
    :param speed: move speed
    :type speed: double
    :param distance: move distance
    :type distance: double
    :param is_forward: move forward or backward, defaults to True
    :type is_forward: bool, optional
    """
    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # get current location from the global variable before entering the loop
    global x, y
    x0 = x
    y0 = y

    # task 1. assign the x coordinate of linear velocity to the speed.
    if is_forward:
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    # we publish the velocity at 10 Hz (10 times a second)
    loop_rate = rospy.Rate(10)

    # task 2. create a publisher for the velocity message on the appropriate topic.
    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=10)

    while True:
        rospy.loginfo("Turtlesim moves forwards")

        # task 3. publish the velocity message
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()

        # rospy.Duration(1.0)

        # measure the distance moved
        distance_moved = distance_moved + \
            abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print(distance_moved)
        if not (distance_moved < distance):
            rospy.loginfo("reached")
            break

    # task 4. publish a velocity message zero to make the robot stop after the distance is reached
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


def rotate(angular_speed_degree, relative_angle_degree, clockwise=True):
    """rotate turtlebot

    :param angular_speed_degree: rotate speed
    :type angular_speed_degree: double
    :param relative_angle_degree: rotate angular
    :type relative_angle_degree: double
    :param clockwise: rotate clockwise / counterclockwise, defaults to True
    :type clockwise: bool, optional
    """
    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # get current location from the global variable before entering the loop
    global yaw
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0

    theta0 = yaw
    angular_speed = math.radians(abs(angular_speed_degree))

    if clockwise:
        velocity_message.angular.z = -abs(angular_speed)    # note clockwise -> negative rotation
    else:
        velocity_message.angular.z = abs(angular_speed)

    # we publish the velocity at 10 Hz (10 times a second)
    loop_rate = rospy.Rate(10)

    # task 2. create a publisher for the velocity message on the appropriate topic.
    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True:
        rospy.loginfo("Turtlesim rotates")

        # task 3. publish the velocity message
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()

        # measure the angle rotated
        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()
        if not (current_angle_degree < relative_angle_degree):
            rospy.loginfo("reached")
            break

    # task 4. publish a velocity message zero to make the robot stop after the distance is reached
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


def move_to_goal(goal_pose, distance_tolerance):
    """go to goal position using proportional control

    :param goal_pose: goal pose
    :type goal_pose: ros msg
    :param distance_tolerance: final distance tolerance
    :type distance_tolerance: double
    """
    global x
    global y, yaw

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    while (True):
        # proportional control linear
        kp_linear = 0.5
        distance = abs(
            math.sqrt(((goal_pose.x-x) ** 2) + ((goal_pose.y-y) ** 2)))

        linear_speed = distance * kp_linear

        # proportional control angular
        kp_angular = 4.0
        desired_angle_goal = math.atan2(goal_pose.y-y, goal_pose.x-x)
        angle = desired_angle_goal-yaw

        angular_speed = angle*kp_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)

        # print velocity_message.linear.x
        # print velocity_message.angular.z
        print('x=', x, 'y=', y)

        if (distance < distance_tolerancejintia and angle < 1):
            break


def set_desired_orientation(desired_angle_degree):
    """[summary]

    :param desired_angle_degree: [description]
    :type desired_angle_degree: [type]
    """
    global yaw

    desired_angle_radians = math.radians(desired_angle_degree)
    relative_angle_radians = desired_angle_radians - yaw

    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print(yaw)
    print(relative_angle_radians)
    print(desired_angle_radians)
    rotate(30, math.degrees(abs(relative_angle_radians)), clockwise)


def grid_clean():
    """

    """
    loop_rate = rospy.Rate(0.5)
    desired_pose = Pose()
    desired_pose.x = 1
    desired_pose.y = 1
    desired_pose.theta = 0

    move_to_goal(desired_pose, 0.01)
    set_desired_orientation(desired_pose.theta)

    move(2.0, 9.0, True)
    loop_rate.sleep()
    rotate(20, 90, False)
    loop_rate.sleep()

    move(2.0, 9.0, True)
    loop_rate.sleep()
    rotate(20, 90, False)
    loop_rate.sleep()

    move(2.0, 1.0, True)
    loop_rate.sleep()
    rotate(20, 90, False)
    loop_rate.sleep()

    move(2.0, 9.0, True)
    loop_rate.sleep()
    rotate(30, 90, True)
    loop_rate.sleep()

    move(2.0, 1.0, True)
    loop_rate.sleep()
    rotate(30, 90, True)
    loop_rate.sleep()

    move(2.0, 9.0, True)


def spiral_clean():
    """[summary]
    """
    # get current location from the global variable before entering the loop
    global x, y

    vel_msg = Twist()
    loop_rate = rospy.Rate(1)
    wk = 4
    rk = 0

    while(x < 10.5 and y < 10.5):
        rk = rk+1
        vel_msg.linear.x = rk
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = wk
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:

        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        # task 5. declare velocity publisher
        velocity_publisher = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)

        time.sleep(2)

        # # move
        # print('move: ')
        # move(1.0, 5.0, True)
        # time.sleep(2)

        # # rotate
        # print('rotate: ')
        # rotate(30, 30, True)
        # time.sleep(2)

        # # grid clean
        # print('grid_clean')
        # grid_clean()
        # time.sleep(2)

        # spiral clean
        print('spiral_clean')
        spiral_clean()
        time.sleep(2)

        # reset
        print('start reset: ')
        rospy.wait_for_service('reset')
        reset_turtle = rospy.ServiceProxy('reset', Empty)
        reset_turtle()
        print('end reset: ')
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
