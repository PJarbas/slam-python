from features import FeatureDetection
from env import Buildenvironment
from sensors import LaserSensor
import random
import pygame

def random_color():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))


if __name__ == "__main__":
    
    feature_map = FeatureDetection()
    
    environment = Buildenvironment((600, 1200))
    # environment.originalmap = environment.map.copy()
    original_map = environment.map.copy()
    
    laser = LaserSensor(200, original_map, uncertainty=(0.5, 0.01))
    environment.map.fill((255, 255, 255))
    environment.infomap = environment.map.copy()
    
    original_map = environment.map.copy()
    
    running = True
    FEATURE_DETECTION = True 
    BREAK_POINT_IND = 0
    
    while running:
        environment.infomap = original_map.copy()
        
        FEATURE_DETECTION = True 
        BREAK_POINT_IND = 0
        END_POINT = [0, 0]
    
        sensor_on = False
        
        PREDICTED_POINTS_TO_DRAW = []
        
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
            
            feature_map.laser_points_set(sensor_data)
            # print(BREAK_POINT_IND, feature_map.NP, feature_map.PMIN)
            
            while BREAK_POINT_IND < (feature_map.NP - feature_map.PMIN):
                
                seed_seg = feature_map.seed_segment_detection(laser.position, BREAK_POINT_IND)
                print(seed_seg)
                if seed_seg == False:
                    break
                else:
                    seed_segment = seed_seg[0]
                    PREDICTED_POINTS_TO_DRAW = seed_seg[1]
                    INDICES = seed_seg[2]
                    results = feature_map.seed_segment_growing(INDICES, BREAK_POINT_IND)
                    
                    if results == False:
                        BREAK_POINT_IND = INDICES[1]
                        continue
                    else:
                        line_eq = results[1]
                        m, c = results[5]
                        line_seg = results[0]
                        OUTER_MOST = results[2]
                        BREAK_POINT_IND = results[3]
                        
                        END_POINT[0] = feature_map.projection_point2line(OUTER_MOST[0], m, c)
                        END_POINT[1] = feature_map.projection_point2line(OUTER_MOST[1], m, c)
                        COLOR = random_color()
                        
                        for point in line_seg:
                            environment.infomap.set_at((int(point[0][0]), int(point[0][1])), (0, 255, 0))
                            pygame.draw.circle(environment.infomap, COLOR, (int(point[0][0]), int(point[0][1])), 2, 0)
                        pygame.draw.line(environment.infomap, (255, 0, 0), END_POINT[0], END_POINT[1], 2)
                            
                        environment.data_storage(sensor_data)
                        # environment.show_sensor_data()
            
        environment.map.blit(environment.infomap, (0, 0))
        pygame.display.update()

