import argparse
from typing import Union

from anomalies import Anomaly
from utils import *
from collections import deque
from sensors import *
from anomalies import *

def main(args):

    client:carla.Client = carla.Client('localhost', 2000)
    world:carla.World = client.get_world()
    tm:carla.TrafficManager = client.get_trafficmanager()

    if args.map is not None:
        world = load_map(world, client, args.map)
    random.seed(args.seed)
    tm.set_random_device_seed(args.seed)

    # In hybrid mode, the physics engine is used only for the vehicles that are in a radious of 20 meters, this is
    # useful for computational efficiency
    if args.hybrid:
        print("Activating hybrid mode")
        tm.set_hybrid_physics_mode(True)
        tm.set_hybrid_physics_radius(20)
    simulation_time_for_tick = 1/args.fps
    set_sync_mode(world,client,simulation_time_for_tick)

    # Set the weather DOESN't WORK
    weather = carla.WeatherParameters(
        cloudiness=args.cloudiness,
        precipitation=args.precipitation,
        wind_intensity=args.wind_intensity,
        sun_altitude_angle=args.sun_altitude_angle,
        fog_density=args.fog_density,
        fog_distance=args.fog_distance,
    )
    world.set_weather(carla.WeatherParameters.HardRainNoon)
    world.tick()

    # Get the spectator
    # the transform guide everything. The location has x,y,z.
    # x is front/back, y is left/right, z is up/down
    spectator:carla.Actor = world.get_spectator()

    for run in range(args.number_of_runs):
        try:

            anomalies = []
            all_vehicles = []
            sensors = []
            ego_vehicle: carla.Actor = spawn_ego_vehicle(world, client, args)
            if ego_vehicle is None:
                raise Exception("Ego vehicle not spawned, exiting...")

            all_vehicles.append(ego_vehicle.id)

            # vehicles is an array of all the ID of the vehicles spawned
            vehicles = generate_traffic(args, world, client)
            all_vehicles.extend(vehicles)

            # The array is composed as follows: [controller, walker, controller, walker, ...]
            contr_walk_array = generate_pedestrian(args, world, client)
            # Spawn the anomalies
            if args.anomalies:
                for str_anomaly in args.anomalies:
                    anomaly_object = generate_anomaly_object(world, client, ego_vehicle, str_anomaly)
                    anomaly: carla.Actor = anomaly_object.spawn_anomaly()
                    if anomaly is None:
                        print(str_anomaly + " -> Tried to spawn anomaly, but it was not spawned. Skipping it...")
                    if anomaly is not None:
                        world.tick()
                        anomalies.append(anomaly_object)
                if len(anomalies) == 0:
                    raise Exception("No anomalies spawned, exiting...")

            # Setup the collision sensor, only if some anomalies are present
            if args.anomalies:
                coll_sen: carla.Sensor = attach_collision_sensor(args, world, client, ego_vehicle)
                coll_queue = deque(maxlen=1)
                coll_sen.listen(lambda data: coll_queue.append(data))
                sensors.append(Collision_Sensor(coll_queue, coll_sen))

            # Set up the sensors
            set_up_sensors(args, client, ego_vehicle, sensors, world)

            #set autopilot for the vehicles and start the simulation
            for vehicle_id in all_vehicles:
                world.get_actor(vehicle_id).set_autopilot(True)

            #With a for, it runs N times the simulation
            # 0.05 is the simulation run for each tick.
            loops = args.run_time / simulation_time_for_tick
            for i in range(int(loops)):
                world.tick()
                if args.anomalies:
                    for anomaly_obj in anomalies:
                        anomaly_obj.tick += 1

                # If the ego vehicle pass the anomaly, break the loop. I compute this in this way:
                # - Get the vector of the difference of the locations
                # - Get the forward vector of the ego vehicle
                # - Compute the dot product of the difference vector and the forward vector
                # Also break the loop if the ego vehicle collides with the anomaly
                if args.anomalies:
                    # Check if the ego vehicle collides with the anomaly
                    if len(coll_queue) != 0:
                        flag = False
                        coll_event:carla.CollisionEvent = coll_queue.pop()
                        for anomaly_obj in anomalies:
                            # Check if the anomaly is the one collided with
                            if coll_event.other_actor.id == anomaly_obj.anomaly.id:
                                print(f"Ego vehicle collided with {anomaly_obj.anomaly}, stopping simulation...")
                                flag = True
                                break
                        if flag:
                            break

                    # Stop if the ego vehicle pass the anomaly
                    # In case of more anomalies, this will stop the simulation when the last anomaly is passed

                    # Compute the furthest anomaly from the ego vehicle and stop if the ego vehicle passes it
                    ego_location = ego_vehicle.get_transform().location
                    more_distance_anomaly = anomalies[0].anomaly
                    more_distance = 0
                    first_iter = True
                    for anomaly_obj in anomalies:
                        # Get the distance vector between the ego vehicle and the anomaly
                        difference = ego_location - anomaly_obj.anomaly.get_transform().location
                        difference_vector = carla.Vector3D(difference.x, difference.y, difference.z)
                        ego_fw = ego_vehicle.get_transform().get_forward_vector()
                        ego_vector = carla.Vector3D(ego_fw.x, ego_fw.y, ego_fw.z)
                        dot = difference_vector.dot(ego_vector)
                        if first_iter:
                            more_distance = dot
                            first_iter = False
                        else:
                            if dot < more_distance:
                                more_distance = dot
                                more_distance_anomaly = anomaly_obj.anomaly
                    # Compute the difference vector and the dot product
                    if more_distance > 0:
                        print(f"Ego vehicle passed the furthest anomaly {more_distance_anomaly}, stopping simulation...")
                        break

                    for anomaly_obj in anomalies:
                        anomaly_obj.handle_semantic_tag()

                # Handle the sensors data
                handle_sensor_data(args, run, sensors)

        except KeyboardInterrupt:
            print("Simulation interrupted by user.")
        except Exception as e:
            print(f"An error occurred: {e}")
            print("Simulation failed. Interrupting...")
        finally:
            # Handle the anomalies
            if args.anomalies:
                for anomaly_obj in anomalies:
                    anomaly_obj.on_destroy()
            clean_up(args,world,client,tm,sensors)
            print("End of simulation")


def handle_sensor_data(args, run, sensors):
    # if the queue is empty, skip one and go to the next tick(). This happens at the first tick() because
    # the data will be available only at the next tick()
    for sensor in sensors:
        sensor.handle(run,args.anomalies)

def set_up_sensors(args, client, ego_vehicle, sensors, world):
    if args.rgb:
        print("Setting up RGB camera")
        rgb_sen: carla.Sensor = attach_rbgcamera(args, world, client, ego_vehicle)
        rgb_queue = deque(maxlen=1)
        rgb_sen.listen(lambda data: rgb_queue.append(data))
        sensors.append(RGB_Sensor(rgb_queue,rgb_sen))
    if args.lidar:
        print("Setting up LIDAR sensor")
        lidar_sen: carla.Sensor = attach_lidar(args, world, client, ego_vehicle)
        lidar_queue = deque(maxlen=1)
        lidar_sen.listen(lambda data: lidar_queue.append(data))
        sensors.append(Lidar_Sensor(lidar_queue,lidar_sen))
    if args.semantic:
        print("Setting up Semantic Segmentation camera")
        semantic_sen: carla.Sensor = attach_semantic_camera(args, world, client, ego_vehicle)
        semantic_queue = deque(maxlen=1)
        semantic_sen.listen(lambda data: semantic_queue.append(data))
        sensors.append(Semantic_Sensor(semantic_queue,semantic_sen))
    if args.lidar_semantic:
        print("Setting up Semantic LIDAR sensor")
        lidar_semantic: carla.Sensor = attach_semantic_lidar(args, world, client, ego_vehicle)
        lidar_semantic_queue = deque(maxlen=1)
        lidar_semantic.listen(lambda data: lidar_semantic_queue.append(data))
        sensors.append(Semantic_Lidar_Sensor(lidar_semantic_queue,lidar_semantic))
    if args.radar:
        print("Setting up Radar sensor")
        radar_sen: carla.Sensor = attach_radar(args, world, client, ego_vehicle)
        radar_queue = deque(maxlen=1)
        radar_sen.listen(lambda data: radar_queue.append(data))
        sensors.append(Radar_Sensor(radar_queue,radar_sen))
    if args.depth:
        print("Setting up Depth camera")
        depth_sen: carla.Sensor = attach_depth_camera(args, world, client, ego_vehicle)
        depth_queue = deque(maxlen=1)
        depth_sen.listen(lambda data: depth_queue.append(data))
        sensors.append(Depth_Sensor(depth_queue,depth_sen))
    if args.instance:
        print("Setting up Instance Segmentation camera")
        instance_sen: carla.Sensor = attach_instance_camera(args, world, client, ego_vehicle)
        instance_queue = deque(maxlen=1)
        instance_sen.listen(lambda data: instance_queue.append(data))
        sensors.append(Instant_Segmentation_Sensor(instance_queue,instance_sen))

def generate_anomaly_object(world, client, ego_vehicle, name):
    if name == "labrador":
        return Labrador_Anomaly(world, client, name, ego_vehicle)
    if name == "baseballbat":
        return Baseballbat_Anomaly(world, client, name, ego_vehicle)
    if name == "basketball":
        return Basketball_Anomaly(world, client, name, ego_vehicle)
    if name == "person":
        return Person_Anomaly(world, client, name, ego_vehicle)
    if name == "tree":
        return Tree_Anomaly(world, client, name, ego_vehicle)
    if name == "beerbottle":
        return Beer_Anomaly(world, client, name, ego_vehicle)
    if name == "football":
        return Football_Anomaly(world, client, name, ego_vehicle)
    if name == "ladder":
        return Ladder_Anomaly(world, client, name, ego_vehicle)
    if name == "mattress":
        return Mattress_Anomaly(world, client, name, ego_vehicle)
    if name == "skateboard":
        return Skateboard_Anomaly(world, client, name, ego_vehicle)
    if name == "tire":
        return Tire_Anomaly(world, client, name, ego_vehicle)
    if name == "woodpalette":
        return WoodPalette_Anomaly(world, client, name, ego_vehicle)
    if name == "basketball_bounce":
        return Basketball_Bounce_Anomaly(world, client, name, ego_vehicle)
    if name == "football_bounce":
        return Football_Bounce_Anomaly(world, client, name, ego_vehicle)
    if name == "streetlight":
        return StreetLight_Anomaly(world, client, name, ego_vehicle)
    if name == "trashcan":
        return TrashCan_Anomaly(world, client, name, ego_vehicle)
    if name == "trafficlight":
        return TrafficLight_Anomaly(world, client, name, ego_vehicle)
    if name == "flippedcar":
        return FlippedCar_Anomaly(world, client, name, ego_vehicle)
    if name == "instantcarbreak":
        return InstantCarBreak_Anomaly(world, client, name, ego_vehicle)
    if name == "trafficlightoff":
        return TrafficLightOff_Anomaly(world, client, name, ego_vehicle)
    if name == "carthroughredlight":
        return CarThroughRedLight_Anomaly(world, client, name, ego_vehicle)
    if name == "roadsigntwisted":
        return RoadSignTwisted_Anomaly(world, client, name, ego_vehicle)
    if name == "roadsignvandalized":
        return RoadSignVandalized_Anomaly(world, client, name, ego_vehicle)
    if name == "motorcycle":
        return Motorcycle_Anomaly(world, client, name, ego_vehicle)
    if name == "garbagebag":
        return GarbageBag_Anomaly(world, client, name, ego_vehicle)
    if name == "garbagebagwind":
        return GarbageBagWind_Anomaly(world, client, name, ego_vehicle)
    if name == "brokenchair":
        return BrokenChair_Anomaly(world, client, name, ego_vehicle)
    if name == "bikes":
        return Bikes_Anomaly(world, client, name, ego_vehicle)
    if name == "hubcap":
        return Hubcap_Anomaly(world, client, name, ego_vehicle)
    print("Anomaly " + name + " not found, returning None")
    return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='CARLA Simulator')
    parser.add_argument("--seed", type=int, help="Random seed", default=42)
    parser.add_argument("--map", type=str, help="Map name to load", default=None)
    parser.add_argument("--number_of_vehicles", type=int, help="Number of vehicles to spawn", default=10)
    parser.add_argument("--number_of_pedestrians", type=int, help="Number of pedestrians to spawn", default=10)
    parser.add_argument("--filterv", type=str, help="Vehicle filter", default="vehicle.*")
    parser.add_argument("--filterw", type=str, help="Pedestrian filter", default="walker.*")
    parser.add_argument("--hybrid", action='store_true', help="Use hybrid physics mode")
    parser.add_argument("--image_size_x", type=str, help="Image size X", default='800')
    parser.add_argument("--image_size_y", type=str, help="Image size Y", default='600')
    parser.add_argument('--sensor_tick', type=str, default='0.1', help='Sensor tick time in seconds (This is the FPS of the sensors)')
    parser.add_argument("--fps", type=int, help="FPS the simulation should run at", default=20)
    parser.add_argument("--rgb", action='store_true', help="Use RGB camera")
    parser.add_argument("--lidar", action='store_true', help="Use lidar sensor")
    parser.add_argument("--radar", action='store_true', help="Use radar sensor")
    parser.add_argument("--depth", action='store_true', help="Use depth sensor")
    parser.add_argument("--lidar_semantic", action='store_true', help="Use semantic lidar sensor")
    parser.add_argument("--semantic", action='store_true', help="Use semantic camera")
    parser.add_argument("--instance", action='store_true', help="Use instance segmentation")
    parser.add_argument("--cloudiness", type=float, help="Use cloudiness", default=0.0)
    parser.add_argument("--precipitation", type=float, help="Use precipitation", default=0.0)
    parser.add_argument("--wind_intensity", type=float, help="Use wind intensity", default=0.0)
    parser.add_argument("--sun_altitude_angle", type=float, help="Use sun altitude angle (Day/Night)", default=0.0)
    parser.add_argument("--fog_density", type=float, help="Use fog density", default=0.0)
    parser.add_argument("--fog_distance", type=float, help="Use fog distance", default=0.0)
    parser.add_argument("--number_of_runs", type=int, help="Number of runs", default=1)
    parser.add_argument("--run_time", type=int, help="Run time in seconds", default=10)
    parser.add_argument("--anomalies", nargs="+", type=str, help="List of anomalies to spawn", default=None)

    args = parser.parse_args()

    main(args)