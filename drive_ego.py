import carla
import random
import logging
import pygame
from pygame.locals import K_w, K_s, K_a, K_d, K_UP, K_DOWN, K_LEFT, K_RIGHT, K_ESCAPE, K_SPACE

logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

def main():
    # Initialize Pygame for keyboard input
    pygame.init()
    display = pygame.display.set_mode((400, 300), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption("CARLA Manual Control")
    font = pygame.font.SysFont("monospace", 15)

    vehicle = None
    try:
        # Connect to CARLA
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        # Setup Vehicle
        bp_lib = world.get_blueprint_library()
        bp = bp_lib.find('vehicle.tesla.model3')
        if bp.has_attribute('role_name'):
            bp.set_attribute('role_name', 'hero')
            
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            logging.error("No spawn points found.")
            return
            
        spawn_point = random.choice(spawn_points)
        vehicle = world.try_spawn_actor(bp, spawn_point)
        if not vehicle:
            logging.error("Failed to spawn vehicle.")
            return
            
        logging.info("Spawned ego vehicle successfully!")
        
        control = carla.VehicleControl()
        clock = pygame.time.Clock()
        
        running = True
        while running:
            clock.tick(60) # 60 FPS update rate
            
            # 1. Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
            keys = pygame.key.get_pressed()
            if keys[K_ESCAPE]:
                running = False
                
            # 2. Compute Control (Throttle / Brake)
            if keys[K_SPACE]: # E-brake
                control.hand_brake = True
                control.throttle = 0.0
                control.brake = 0.0
            else:
                control.hand_brake = False
                if keys[K_w] or keys[K_UP]:
                    control.throttle = min(control.throttle + 0.1, 1.0)
                    control.brake = 0.0
                    control.reverse = False
                elif keys[K_s] or keys[K_DOWN]:
                    # Simple reverse logic
                    v = vehicle.get_velocity()
                    speed = (v.x**2 + v.y**2 + v.z**2)**0.5
                    if speed < 0.1 and not control.reverse:
                        control.reverse = True
                        
                    if control.reverse:
                        control.throttle = min(control.throttle + 0.1, 1.0)
                        control.brake = 0.0
                    else:
                        control.brake = min(control.brake + 0.2, 1.0)
                        control.throttle = 0.0
                else:
                    control.throttle = 0.0
                    control.brake = 0.0
                
            # 3. Compute Control (Steering)
            steer_increment = 0.05
            if keys[K_a] or keys[K_LEFT]:
                control.steer = max(control.steer - steer_increment, -1.0)
            elif keys[K_d] or keys[K_RIGHT]:
                control.steer = min(control.steer + steer_increment, 1.0)
            else:
                control.steer = 0.0 # Snap back to center
                
            # 4. Apply Control
            vehicle.apply_control(control)
            
            # 5. Render UI
            display.fill((30, 30, 30))
            lines = [
                "CARLA Manual Control",
                "--------------------",
                "W / UP    : Accelerate",
                "S / DOWN  : Brake/Reverse",
                "A / D     : Steer",
                "SPACE     : Handbrake",
                "ESC       : Quit",
                "",
                f"Throttle : {control.throttle:.2f}",
                f"Brake    : {control.brake:.2f}",
                f"Steer    : {control.steer:.2f}",
                f"Reverse  : {control.reverse}"
            ]
            for i, text in enumerate(lines):
                surface = font.render(text, True, (255, 255, 255))
                display.blit(surface, (20, 20 + i * 20))
            pygame.display.flip()
            
    except Exception as e:
        logging.error(f"Error: {e}")
    finally:
        if vehicle is not None:
            vehicle.destroy()
            logging.info("Vehicle destroyed.")
        pygame.quit()

if __name__ == '__main__':
    main()
