from env import Buildenvironment
from sensors import LaserSensor
from features import FeatureDetection
import pygame
import math
import random

# source: https://www.youtube.com/watch?v=JbUNsYPJK1U&list=PL9RPomGb9IpRJLw5UTdSy4eJeoLrwNcfC&index=2


if __name__ == "__main__":
    environment = Buildenvironment((600, 1200))
    environment.originalmap = environment.map.copy()
    laser = LaserSensor(200, environment.originalmap, uncertainty=(0.5, 0.01))
    environment.map.fill((0, 0, 0))
    environment.infomap = environment.map.copy()
    
    running = True
    
    while running:
        sensor_on = False
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if pygame.mouse.get_focused():
                sensor_on = True
            elif not pygame.mouse.get_focused():
                sensor_on = False
        
        if sensor_on:
            position = pygame.mouse.get_pos()
            laser.position = position
            sensor_data = laser.sense_obstacles()
            environment.data_storage(sensor_data)
            environment.show_sensor_data()
            
        environment.map.blit(environment.infomap, (0, 0))
        pygame.display.update()
