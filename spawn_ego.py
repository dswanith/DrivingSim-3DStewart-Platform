import carla
import random
import logging

logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

def main():
    try:
        # Connect to CARLA server
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        # Get the blueprint library and pick a vehicle
        blueprint_library = world.get_blueprint_library()
        # Let's use a Tesla Model 3
        bp = blueprint_library.find('vehicle.tesla.model3')
        
        # VERY IMPORTANT: The driving simulator looks for role_name='hero' or 'ego_vehicle'
        if bp.has_attribute('role_name'):
            bp.set_attribute('role_name', 'hero')
            
        # Find a valid spawn point
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            logging.error("Could not find any spawn points in this map.")
            return
            
        spawn_point = random.choice(spawn_points)
        
        # Spawn the vehicle
        vehicle = world.try_spawn_actor(bp, spawn_point)
        if vehicle is None:
            logging.error("Failed to spawn vehicle at the chosen location. Try again.")
            return
            
        logging.info(f"Successfully spawned ego vehicle: {vehicle.type_id} (ID: {vehicle.id})")
        
        # Turn on autopilot so it drives around automatically!
        vehicle.set_autopilot(True)
        logging.info("Autopilot engaged. The car is now driving itself around the map.")
        logging.info("Press Ctrl+C to destroy the vehicle and exit.")
        
        # Keep script running until user interrupts
        while True:
            world.wait_for_tick()
            
    except KeyboardInterrupt:
        logging.info("Interrupted by user. Cleaning up...")
    except Exception as e:
        logging.error(f"Error: {e}")
        logging.error("Is the CARLA simulator server running? (e.g. CarlaUE4.exe)")
    finally:
        if 'vehicle' in locals() and vehicle is not None:
            vehicle.destroy()
            logging.info("Vehicle destroyed.")

if __name__ == '__main__':
    main()
