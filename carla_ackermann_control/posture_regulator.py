#!/usr/bin/python3
# boh
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf

# Gains
k1_base = 1.8
k2_base = 1
k3_base = 1

# Thresholds
thresholdM = 1 
thresholdR = 0.1

# Vehicle parameters
l = 2.9  # Distance between the front and rear axles
max_steering_angle = math.radians(35)  # Physical limit of tesla model 3

# State variables
current_index = 2 # Set to 2 to discard fewer superfluous points that are too close to the vehicle at the beginning
path_points = []
current_pose = None
xd, yd, yawd, yawd_prev = None, None, 0, 0
x, y, yaw, yaw_prev = None, None, 0, 0
path_actual = []
velocities = []
simulation_complete = False
steering_angles = []

def normalize_angle(angle): # To prevent angles from going beyond +- pi (since all angles are defined between -pi and +pi)
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def callback_current_pose(data):
    global current_pose, x, y, yaw, yaw_prev, yawd, simulation_complete

    if simulation_complete:
        return
    
    current_pose = data.pose.pose  # Extract current pose from PoseWithCovariance
    x = current_pose.position.x
    y = current_pose.position.y
    orientation_q = current_pose.orientation

    # Convert quaternion to Euler angles
    quaternion = (
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw_prev = yaw
    yaw = euler[2]

    # Store actual path
    path_actual.append((x, y))
    
    rospy.loginfo("Current vehicle pose: x=%f, y=%f, yaw=%f", x, y, yaw)


def callback_waypoints(data):
    global path_points
    path_points = data.poses
    rospy.loginfo("Received %d path points", len(path_points))

def process_point():
    global current_index, xd, yd, yawd, yawd_prev
    if current_index < len(path_points):
        pose = path_points[current_index].pose
        xd = pose.position.x
        yd = pose.position.y
        orientation_q = pose.orientation

        # Convert quaternion to Euler angles
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yawd = euler[2]
        rospy.loginfo("Path point: xd=%f, yd=%f, yawd=%f", xd, yd, yawd)
    else:
        rospy.loginfo("All path points have been processed.")

def is_approaching_curve(lookahead=15): # To detect when a vehicle is about to approach a curve (threshold of 0.1 rad, i.e. thresoldR variable at the beginning of the controller's code)
    global current_index, path_points, yaw

    if current_index + lookahead < len(path_points):
        next_pose = path_points[current_index + lookahead].pose

        orientation_q = next_pose.orientation

        # Convert quaternion to Euler angles
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        next_yawd = euler[2]
        yaw_difference = abs(abs(next_yawd) - abs(yaw))

        # Define a threshold to detect a curve
        curve_threshold = 0.1  # Radians
        return yaw_difference > curve_threshold
    return False

def ackermann_controller():
    global rho, gamma, delta, thresholdM, thresholdR, k1

    rho = math.sqrt((xd - x)**2 + (yd - y)**2)
    gamma = normalize_angle(math.atan2((yd - y), (xd - x)) - yaw)
    delta = gamma - normalize_angle(yawd - yaw)

    if current_index + 5 > len(path_points): # To slow down the vehicle when it approaches the end of the path
        k1 = 0.7
        thresholdM = 0.25
    else:
    # Adjust gains dynamically based on distance and angle
        k1 = k1_base * abs(rho)
    k2 =  k2_base * abs(gamma)**3
    k3 =  k3_base * abs(delta)**3

    # Reduce speed and adjust thresholds during turns
    if is_approaching_curve() or abs(yaw - yawd) > 0.1: 
        thresholdM = 1
        thresholdR = 0.5 # When the vehicle is about to approach a curve or when it is inside one, we have chosen to increase the angular threshold so that points are taken a little further apart from each other to facilitate the maneuver (k1, k2 and k3 higher, as well as v and phi)
        k1 *= 0.8 # Reduce speed during turns
        k2 *= 3 # Increase of phi in the curves
        k3 *= 3 # Increase of phi in the curves

    # Calculate control actions
    c1 = k2 * gamma + k1 * (math.sin(gamma) * np.sign(math.cos(gamma))) / gamma * (gamma + k3 * delta)
    v = k1 * rho * np.sign(math.cos(gamma))
    c2 = l * c1 / v
    phi = math.atan(c2) # Equivalence with the “arccosine formula” (see section 2)

    # Limit the steering angle to the vehicle's physical limits
    phi = max(-max_steering_angle, min(max_steering_angle, phi)) 

    # Store velocity and steering angle
    velocities.append(v)
    steering_angles.append(np.degrees(phi))
    rospy.loginfo("V: v=%f", v)
    rospy.loginfo("V: phi=%f", phi)

    omega = (v/l)*math.tan(phi)

    return v, omega

def stop_vehicle(pub): 
    stop_cmd = Twist()
    stop_cmd.linear.x = 0
    stop_cmd.angular.z = 0
    pub.publish(stop_cmd)
    rospy.loginfo("Vehicle has stopped.")

def plot_results():
    global path_points, path_actual, velocities, steering_angles

    # Reference trajectory
    ref_x = [p.pose.position.x for p in path_points]
    ref_y = [p.pose.position.y for p in path_points]

    # Activate interactive mode
    plt.ion()
    # Actual path
    actual_x, actual_y = zip(*path_actual)
    
    # Plot
    plt.figure(figsize=(10, 10))
    plt.plot(ref_x, ref_y, 'r-', label='Reference Path')
    plt.plot(actual_x, actual_y, 'b', label='Vehicle Path')
    plt.scatter(ref_x[0], ref_y[0], color='green', s=100, label='Starting point')
    plt.scatter(ref_x[-1], ref_y[-1], color='purple', s=100, label='Goal point')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Paths')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Control actions plot
    plt.figure(figsize=(12, 6))
    # Velocity plot
    plt.subplot(2, 1, 1)
    plt.plot(velocities, 'g', label='Velocity')
    plt.xlabel('Time (frame)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Velocity')
    plt.legend()
    plt.grid(True)
    # Steering angle plot
    plt.subplot(2, 1, 2)
    plt.plot(steering_angles, 'm', label='Steering angle (phi)')
    plt.xlabel('Time (frame)')
    plt.ylabel('Steering angle (°)')
    plt.title('Steering angle')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    plt.draw()  # Update the plot
    plt.pause(0.001)  # Necessary to make the plot visible
    input("Press Enter to close the plots...")
    plt.close('all')  # Closes all figures

def main():
    global current_index, path_points, simulation_complete

    rospy.init_node('posture_regulator', anonymous=True)
    rospy.Subscriber('/carla/ego_vehicle/waypoints', Path, callback_waypoints)
    rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, callback_current_pose)

    pub = rospy.Publisher('/carla/ego_vehicle/twist', Twist, queue_size=10)
    rate = rospy.Rate(50) # Frequency of 50 Hz (controller), slightly upper than odometry (topic) frequency 

    while not rospy.is_shutdown():
        if path_points:
            process_point()
            if x is not None and y is not None and yaw is not None and xd is not None and yd is not None and yawd is not None:
                while abs(xd - x) > thresholdM or abs(yd - y) > thresholdM or abs(abs(yawd) - abs(yaw)) > thresholdR: # The angular condition with the "abs" function is used to avoid ambiguity when yaw is on the max/min threshold of the angular definition (+pi or -pi)
                    v, omega = ackermann_controller()
                    vel_cmd = Twist()
                    vel_cmd.linear.x = v 
                    vel_cmd.angular.z = omega

                    # Publish the control inputs
                    pub.publish(vel_cmd)
                    rate.sleep()
                current_index += 1
                if current_index >= len(path_points):
                    rospy.loginfo("All path points have been processed.")
                    stop_vehicle(pub)
                    simulation_complete = True
                    break
        else:
            rospy.loginfo("Waiting to receive path points...")
        rate.sleep()

    if simulation_complete:
        rospy.loginfo("Simulation complete. Preparing to plot results.")
        plot_results()
        rospy.signal_shutdown("Simulation finished.")
        
if __name__ == '__main__':
    main()