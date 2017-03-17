from assignment_3.geometry import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped
from math import sin, cos, degrees
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# ------------------------------------------------------------------------------
# MapMaker class
# ------------------------------------------------------------------------------
class MapMaker:
  def __init__(self, origin_x, origin_y, resolution, size_x, size_y, transformer):
    self.origin_x = origin_x
    self.origin_y = origin_y
    self.resolution = resolution
    self.size_x = size_x
    self.size_y = size_y
    self.transformer = transformer
    
    self.grid = OccupancyGrid()
    self.grid.header.frame_id = 'odom'
    self.grid.info.resolution = resolution
    self.grid.info.width = size_x
    self.grid.info.height = size_y
    self.grid.info.origin.position.x = origin_x
    self.grid.info.origin.position.y = origin_y
    self.grid.info.origin.orientation.w = 1.0
    self.grid.data = [-1] * (size_x * size_y)
    self.numScansReceived = 0
    self.pose = [0,0,0];
    self.current_pose = [0,0,0];
    self.received = False;
    # Insert additional code here if needed

  # ----------------------------------------------------------------------------
  # Convert from world coordinates to grid coordinates. This is convenience 
  # wrapper around the to_grid function from the first part of the assignment.
  # Usage:
  #   (x_grid, y_grid) = self.to_grid(x_world, y_world)
  def to_grid(self, x, y):
    return to_grid(x, y, self.origin_x, self.origin_y, self.size_x, self.size_y, self.resolution)    

  # ----------------------------------------------------------------------------
  # Convert from grid coordinates to world coordinates. This is convenience 
  # wrapper around the to_world function from the first part of the assignment.
  # Usage:
  #   (x_world, y_world) = self.to_world(x_grid, y_grid)
  def to_world(self, gx, gy):
    return to_world(gx, gy, self.origin_x, self.origin_y, self.size_x, self.size_y, self.resolution)    

  # ----------------------------------------------------------------------------
  # Process odometry message. You code should go here.
  def process_odom(self, msg):
    rob_x = msg.pose.pose.position.x;
    rob_y = msg.pose.pose.position.y;
    orientation = msg.pose.pose.orientation;
    orientation = (orientation.x, orientation.y, orientation.z, orientation.w);
    theta = euler_from_quaternion(orientation)[2];
    self.current_pose = [rob_x,rob_y,theta]
    self.received = True;
    None

  # ----------------------------------------------------------------------------
  # Process laserscan message. You code should go here.
  def process_scan(self, msg):
    if self.received:
    #print('In process scan')
      current_pose = self.current_pose;
      angle_min = msg.angle_min
      angle_max = msg.angle_max
      angle_increment = msg.angle_increment
      range_max = msg.range_max
      range_min = msg.range_min
      [rob_grid_x,rob_grid_y] = self.to_grid(current_pose[0],current_pose[1])
      ranges = msg.ranges
    
      cnt = 0;
      for obs_dist in ranges:
        i = angle_min + angle_increment*cnt;
        cnt+=1;
      
        obs_tetha = i + current_pose[2];
        obs_world_x = current_pose[0] + obs_dist*cos(obs_tetha);
        obs_world_y = current_pose[1] + obs_dist*sin(obs_tetha);
        obs_grid = self.to_grid(obs_world_x,obs_world_y);
  
        if obs_grid is not None:
          obs_grid_x = obs_grid[0];
          obs_grid_y = obs_grid[1];
          ray = bresenham(rob_grid_x,rob_grid_y,obs_grid_x,obs_grid_y);
          for indx in ray[1:len(ray)-1]:
            occupancy_grid_indx = to_index(indx[0],indx[1],self.size_x)
            if self.grid.data[occupancy_grid_indx] < 0:
              self.grid.data[occupancy_grid_indx] = 0;
          if obs_dist > range_min and obs_dist < range_max:
            occupancy_grid_indx = to_index(obs_grid_x,obs_grid_y,self.size_x);
            self.grid.data[occupancy_grid_indx] = 100;
      self.numScansReceived+=1
    None        

