import pygame
import math
import numpy as np 


def uncertainty_add(distance, angle, sigma):
    mean = np.array([distance, angle])
    covariance = np.diag(sigma**2)
    distance, angle = np.random.multivariate_normal(mean, covariance)
    distance = max(distance, 0)
    angle = max(angle, 0)
    return [distance, angle]

class LaserSensor:
    def __init__(self, range, map, uncertainty, speed=4):
        
        self.range = range
        
        # fix speed for the sensors rotation, rounds per second
        self.speed = speed 
        self.sigma = np.array([uncertainty[0], uncertainty[1]])
        self.position = (0, 0)
        self.map = map
        self.w, self.h = pygame.display.get_surface().get_size()
        self.sensed_obstacles = []
        
    def distance(self, obstacle_position):
        px = (obstacle_position[0] - self.position[0])**2
        py = (obstacle_position[1] - self.position[1])**2
        return math.sqrt(px+py)
    
    def sense_obstacles(self):
        
        data = []
        x1, y1 = self.position[0], self.position[1]
        
        for angle in np.linspace(0, 2*math.pi, 60, False):
        
            x2, y2 = (x1 + self.range*math.cos(angle), y1 + self.range*math.sin(angle))
        
            for i in range(0, 100):
                u = i/100
                x = int(x2*u + x1*(1-u))
                y = int(y2*u + y1*(1-u))
        
                if 0 < x < self.w and 0 < y < self.h:
                    color = self.map.get_at((x, y))
                    
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        
                        distance = self.distance((x, y))
                        
                        output = uncertainty_add(distance, angle, self.sigma)
                        output.append(self.position)
                        
                        data.append(output)
                        break
                    
            if len(data) > 0:
                return data
            else:
                return []
        
        
        