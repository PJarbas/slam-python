import pygame
import math
import numpy as np


def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)


class Robot:
    def __init__(self, startpos, width):
        
        self.meters_to_pixel = 3779.52
        
        self.w = width
        self.x = startpos[0]
        self.y = startpos[1]
        
        self.heading = 0

        self.velocity_l = 0.01*self.meters_to_pixel
        self.velocity_r = 0.01*self.meters_to_pixel
        
        self.max_speed = 0.02*self.meters_to_pixel
        self.min_speed = 0.01*self.meters_to_pixel
        
        self.min_obs_dist = 100
        self.count_down = 5
        
        
    def avoid_obstacles(self, point_cloud, time_step):
        closest_obs = None
        dist = np.inf
        
        if len(point_cloud) > 1:
            for point in point_cloud:
                if dist > distance([self.x, self.y], point):
                    dist = distance([self.x, self.y], point)
                    closest_obs = (point, dist)

            if closest_obs[1] < self.min_obs_dist and self.count_down > 0:
                self.count_down -= time_step
                self.move_backward()
            else:
                self.count_down = 5
                self.move_forward()
    
    def move_backward(self):
        self.velocity_r = -self.min_speed
        self.velocity_l = -self.min_speed/2
        
    def move_forward(self):
        self.velocity_r = self.min_speed
        self.velocity_l = self.min_speed
        
    def kinematics(self, time_step):
        
        self.x += ((self.velocity_l + self.velocity_r)/2) * math.cos(self.heading) *time_step
        self.y -= ((self.velocity_l + self.velocity_r)/2) * math.sin(self.heading) *time_step
        
        self.heading += (self.velocity_r - self.velocity_l) / self.w * time_step
        
        if self.heading > 2*math.pi or self.heading < -2*math.pi:
            self.heading = 0
        
        self.velocity_r = max(min(self.max_speed, self.velocity_r), self.min_speed)
        self.velocity_l = max(min(self.max_speed, self.velocity_l), self.min_speed)
            
             
                    
                