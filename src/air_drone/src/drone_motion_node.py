#!/usr/bin/env python3
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import rospy
import numpy as np
from tf.transformations import *
from air_drone.msg import Pose, MotorSpeed
#from turtle import position
import gym
from gym import spaces
import rospy
from sensor_msgs.msg import Image


import cv2
from stable_baselines3 import DQN
from std_srvs.srv import Empty
from rospy.exceptions import ROSTimeMovedBackwardsException
import random
import math

### Publishers: 
# MotorSpeed: "motor_speed_cmd"
# Marker: "/goal_marker"

### Subscribers:
# Image: "depth_camera/depth/image_raw"
# Pose: "pose" 

### Service:
# Empty: "/gazebo/reset_simulation"

class PointMassNavigationEnv(gym.Env):
    def __init__(self):
        # initializations
        self.time_step = 0 #for calculating time penalty in the reward
        # motor speeds at first step
        self.a = 20.0 
        self.b = 20.0 
        self.c = 20.0
        self.d = 20.0
        self.e = 20.0
        self.motor_const = [self.a, self.b, self.c, self.d, self.e]
        self.pose_x, self.pose_y, self.pose_z = 0.0, 0.0, 0.0
        self.goal_x, self.goal_y, self.goal_z = 2.0, 2.0, 1.0 
        self.reset_con = False         
        self.reward = 0.0
        self.current_depth_map = None
        self.camera_data = None
        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(low=0, high=255, shape=(480, 640, 1), dtype=np.uint8)
        self.action = 0
        # Publishers and subscribers
        self.image_sub = rospy.Subscriber("depth_camera/depth/image_raw", Image, self.camera_callback) # depth camera
        self.current_pose_sub = rospy.Subscriber("pose", Pose, self.position_callback)                 # current drone position 
        self.speed_motors_pub = rospy.Publisher("motor_speed_cmd", MotorSpeed, queue_size=1)           # motor speed 
        self.reset_simulation_service = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)          # resets gazebo  
        self.marker_pub = rospy.Publisher("/goal_marker", Marker, queue_size=1)                        # marks goal point 
        

    def publish_goal_marker(self):
        # Create a marker message
        marker = Marker()
        marker.header.frame_id = "drone"  # Set the frame id
        marker.id = 0  # Marker ID
        marker.type = Marker.SPHERE  # Marker type (SPHERE)
        marker.action = Marker.ADD  # Action (ADD)
        marker.pose.position = Point(self.goal_x, self.goal_y, self.goal_z)  # Set the position of the marker
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Set marker scale
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # Set alpha (transparency)
        marker.color.r = 0.0  
        marker.color.g = 0.0
        marker.color.b = 1.0  # Set color (blue)  
        self.marker_pub.publish(marker)
      

    def generate_random_goal(self, x_range=(-5.0, 5.0), y_range=(-5.0, 5.0), z_range=(0.5, 1.0)):
        self.goal_x = random.uniform(x_range[0], x_range[1])
        self.goal_y = random.uniform(y_range[0], y_range[1])
        self.goal_z = random.uniform(z_range[0], z_range[1])
        self.goal_x = 3.0
        self.goal_y = 3.0
        self.goal_z = 3.0
        #rospy.loginfo(f"Goal x, y, z: {self.goal_x},{self.goal_y},{self.goal_z}")

    
    def position_callback(self, pos_msg):
        # Current position of the drone
        self.pose_x, self.pose_y, self.pose_z = pos_msg.x, pos_msg.y, pos_msg.z
        self.pitch, self.roll = pos_msg.pitch, pos_msg.roll
    
    
    def camera_callback(self, image_msg):
        # Handling for depth camera data 
        try:
            depth_data = np.frombuffer(image_msg.data, dtype=np.float32)
            depth_image = depth_data.reshape((image_msg.height, image_msg.width))
            # Handle NaN values by replacing them with zeros
            depth_image = np.nan_to_num(depth_image)
  
            epsilon = 1e-5
            denominator = (depth_image.max() - depth_image.min())
            if denominator == 0:
                denominator = epsilon  # Avoid division by zero
            normalized_depth = ((depth_image - depth_image.min()) / denominator * 255).astype(np.uint8)

            #self.camera_data = cv2.cvtColor(normalized_depth, cv2.COLOR_GRAY2RGB)
            self.current_depth_map = normalized_depth
            self.camera_data = normalized_depth
            #rospy.loginfo(f"Depth image min value: {self.current_depth_map}")
       
        except ValueError as ve:
            rospy.logerr(f"ValueError processing camera data: {ve}")
        except IndexError as ie:
            rospy.logerr(f"IndexError processing camera data: {ie}")
        except Exception as e:
            rospy.logerr(f"Error processing camera data: {e}")
    

    def step(self, action):
        self.time_step += 1
        # Take the action
        self.motor_speeds = [[self.motor_const[0], -self.motor_const[0], 0.0, -self.motor_const[0]/2, self.motor_const[0]/2, 0.0],
                             [self.motor_const[1]/2, -self.motor_const[1], self.motor_const[1], -self.motor_const[1], self.motor_const[1]/2, -self.motor_const[1]/2],
                             [self.motor_const[2], -self.motor_const[2]/2, self.motor_const[2], -self.motor_const[2]/2, self.motor_const[2], -self.motor_const[2]/2],
                             [self.motor_const[3], -self.motor_const[3], self.motor_const[3], -self.motor_const[3], self.motor_const[3], -self.motor_const[3]],
                             [-self.motor_const[4], self.motor_const[4], -self.motor_const[4], self.motor_const[4], -self.motor_const[4], self.motor_const[4]]]
        #Actions: 
        self.vel_cmd = MotorSpeed()
        if action == 0: #roll
            self.vel_cmd.name = ['propeller1','propeller2','propeller3','propeller4','propeller5','propeller6']
            self.vel_cmd.velocity = self.motor_speeds[0]
        elif action == 1: #pitch
            self.vel_cmd.name = ['propeller1','propeller2','propeller3','propeller4','propeller5','propeller6']
            self.vel_cmd.velocity = self.motor_speeds[1]
        elif action == 2: #yaw
            self.vel_cmd.name = ['propeller1','propeller2','propeller3','propeller4','propeller5','propeller6']
            self.vel_cmd.velocity = self.motor_speeds[2]
        elif action == 3:#up
            self.vel_cmd.name = ['propeller1','propeller2','propeller3','propeller4','propeller5','propeller6']
            self.vel_cmd.velocity = self.motor_speeds[3]
        elif action == 4:#down
            self.vel_cmd.name = ['propeller1','propeller2','propeller3','propeller4','propeller5','propeller6']
            self.vel_cmd.velocity = self.motor_speeds[4]
               
        #rospy.loginfo(f"Action number: {action}")  
        #rospy.loginfo(f"Velocity message: {self.vel_cmd.velocity}")  
        self.speed_motors_pub.publish(self.vel_cmd)

        
        # Calculate reward based on camera data
        #self.observation = np.repeat(self.current_depth_map[:, :, np.newaxis], 3, axis=2)  # Expand to 3 channels 
        self.observation = self.current_depth_map[:, :, np.newaxis]  # Add a new axis to create shape (480, 640, 1)
        self.reward = self.calculate_reward()
        self.done = False 
        self.info = {}

        return self.observation, self.reward, self.done, self.info
    

    def reset(self):
        # Reset the environment
        self.camera_data = np.zeros((480, 640, 1)) 
        return self.camera_data


    def check_collision(self):
        if self.current_depth_map is None:
            return False
       
        if abs(self.roll) > math.radians(170) or abs(self.pitch) > math.radians(170):
            print("----------------------upside down-----------------------")
            print("Reset: roll/ pitch angle is too high")
            self.drone_reset = True
        elif abs(self.roll) > math.radians(87) and abs(self.roll) < math.radians(93):  
            self.drone_reset = True   
            print("drone is vercital")
        elif abs(self.pose_x) > 5.0 or abs(self.pose_y) > 5.0:
            self.drone_reset = True 
            print("drone is out of range")
        elif self.pose_z > 10.0:
            self.drone_reset = True 
            print("Drone height is too much")   
        else:
            self.drone_reset = False

        if self.drone_reset:
            return True # Collision detected
    
        return False  # No collision detected
    


    def adjust_motor_speeds(self,action_index):
        adjusted_speeds = (self.motor_const[action_index]) +  9*self.reward_improvement
        self.motor_const[action_index] = np.clip(adjusted_speeds,-200.0,200.0) 


    def calculate_reward(self):
        # Calculate reward based on camera data
        if self.camera_data is None:
            return 0.0
       
        time_penalty_coeff = -0.01
        time_penalty = time_penalty_coeff * self.time_step

        target_distance_threshold = 0.5  # Threshold for considering target reached
        collision_penalty = -10000.0     # Penalty for collisions
        target_reward = 10000.0          # Reward for reaching the target      

        current_position = [self.pose_x, self.pose_y, self.pose_z]
        goal_position = [self.goal_x, self.goal_y, self.goal_z]
        distance_to_goal = np.linalg.norm(np.array(current_position) - np.array(goal_position))
        distance_reward = - distance_to_goal

        collided = self.check_collision()
        #Calculate the reward
        if collided:
           distance_reward = -distance_to_goal + collision_penalty
           self.reset_con = True
        elif distance_to_goal < target_distance_threshold:
           self.reward = target_reward
           self.reset_con = False
           print("***********************GOAL REACHED*****************************")
           self.drone_reset = True
        else:
            self.reset_con = False
            distance_reward = -distance_to_goal
            
        # Sum up different reward terms
        velocity_reward = -np.linalg.norm(np.array(self.vel_cmd.velocity))/100
        prev_reward = self.reward
        self.reward = distance_reward + time_penalty + velocity_reward
        self.reward_improvement = self.reward - prev_reward
        self.adjust_motor_speeds(self.action)
        #print("Reward: "+str(self.reward))
        return self.reward 
  
    def main_loop(self):
        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():  
             
            # Initialize the DQN model
            model = DQN('CnnPolicy', env, learning_rate = 0.001, buffer_size=10000, exploration_fraction=0.1,verbose=1)
            self.observation = env.reset()
            total_timesteps = 1000
            self.generate_random_goal()
            self.publish_goal_marker() 
            rospy.loginfo(f"goal point: {self.goal_x}, {self.goal_y}, {self.goal_z}")
            
            for timestep in range(total_timesteps):
                
                #print(timestep)    
                if self.reset_con and timestep>0:
                    # Trigger reset logic
                    rospy.loginfo(f"-----------------------RESET--------------------------------")
                    try:
                        self.reset_simulation_service()  # Call the reset simulation service  
                    except ROSTimeMovedBackwardsException:
                        rospy.logwarn("ROSTimeMovedBackwardsException caught. Delaying reset...")
                        current_time = rospy.Time.now()
                        while rospy.Time.now() - current_time < rospy.Duration(1.0):
                            self.rate.sleep()
                    self.observation = env.reset()  # Reset the environment observation 
                    self.generate_random_goal()
                    self.publish_goal_marker()                  
                    self.is_upside_down = False  # Reset the flag
                    self.reset_con = False
                    self.last_ros_time = rospy.Time.now()
                    #self.rate.sleep()
           
                else:    
                    self.action, _ = model.predict(self.observation)
                    self.observation = self.current_depth_map[:, :, np.newaxis]  # Convert to (480, 640, 1)
                    self.observation, self.reward, self.done, self.info = env.step(self.action)
                    model.learn(total_timesteps=1, reset_num_timesteps=False) 
                    print(f"Timestep: {timestep}, Action: {self.action}, Reward: {self.reward}")
                    

                    #self.rate.sleep()
                    if self.done:
                        self.observation = env.reset()
         
                
            self.rate.sleep()
            model.save("point_mass_navigation_dqn")


if __name__ == '__main__':
    rospy.init_node('drone_motion_node')
    env = PointMassNavigationEnv()
    env.main_loop()
