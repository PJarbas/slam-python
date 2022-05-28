import math
import pygame
from robot import Robot
from graphics import Graphics
from ultrasonic import Ultrasonic


MAP_DIMENSIONS = (675, 1200)
ROBOT_PATH = "robot-vacuum.png"
MAP_PATH = "map.png"

if __name__ == "__main__":
    
    gfx = Graphics(MAP_DIMENSIONS, ROBOT_PATH, MAP_PATH)
    
    # robot
    start = (200, 200)
    robot = Robot(start, 0.01*3779.52)
    
    # sensor
    sensor_range = 250, math.radians(40)
    
    ultra_sonic = Ultrasonic(sensor_range, gfx.map)
    
    time_step = 0
    last_time = pygame.time.get_ticks()
    
    running = True
    
    # simulation loop
    while running:
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        time_step = (pygame.time.get_ticks() - last_time)/1000
        last_time = pygame.time.get_ticks()
        
        gfx.map.blit(gfx.map_img, (0, 0))
        
        robot.kinematics(time_step)
        
        gfx.draw_robot(robot.x, robot.y, robot.heading)
        
        point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading)
        
        robot.avoid_obstacles(point_cloud, time_step)
        
        gfx.draw_sensor_data(point_cloud)
        
        pygame.display.update()
                
          
    
    
    
    
    