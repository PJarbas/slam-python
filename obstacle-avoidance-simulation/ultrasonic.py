import numpy as np
import pygame
import math


class Ultrasonic:
    def __init__(self, sensor_range, map):
        self.sensor_range = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.map = map
    
    def sense_obstacles(self, x, y, heading):
        obstacles = []
        x1, y1 = x, y
        
        start_angle = heading - self.sensor_range[1]
        finish_angle = heading + self.sensor_range[1]
        
        for angle in np.linspace(start_angle, finish_angle, 10, False):
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = y1 - self.sensor_range[0] * math.sin(angle)
            
            for i in range(0, 100):
                u = i/100
                x = int(x2 * u + x1 * (1-u))
                y = int(y2 * u + y1 * (1-u))

                if 0 < x < self.map_width and 0 < y < self.map_height:
                    
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 208, 255))
                    
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        obstacles.append([x, y])
                        break
        
        return obstacles
                    
                    
            