import argparse
from queue import Queue, Empty

from sensors import *
from anomalies import *
import logging
from logging import info,warning,error

def main(args, bench_run=None):
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s %(levelname)s: %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S',
    )
    if args.log:
        info("Run: %s", bench_run)
        info("Seed: %s", args.seed)
        #info("Anomalies: %s", args.anomalies)
        info("Number of vehicles: %s", args.number_of_vehicles)
        info("Number of pedestrians: %s", args.number_of_pedestrians)
        info("Semantic sensor: %s", args.semantic)
        info("RGB sensor: %s", args.rgb)
        info("Depth sensor: %s", args.depth)
        info("Lidar sensor: %s", args.lidar)
        info("Lidar Semantic sensor: %s", args.lidar_semantic)
        info("Radar sensor: %s", args.radar)
        info("Instance Segmentation sensor: %s", args.instance)
        info("FPS: %s", args.fps)
        info("Sensor tick: %s", args.sensor_tick)
        info("Hybrid mode: %s", args.hybrid)

    client:carla.Client = carla.Client('localhost', 2000)
    world:carla.World = client.get_world()
    tm:carla.TrafficManager = client.get_trafficmanager()

    if args.map is not None:
        world = load_map(world, client, args.map)

    tm.set_random_device_seed(args.seed)

    # In hybrid mode, the physics engine is used only for the vehicles that are in a radious of 50 meters, this is
    # useful for computational efficiency
    if args.hybrid:
        info("Activating hybrid mode")
        tm.set_hybrid_physics_mode(True)
        tm.set_hybrid_physics_radius(50)
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
    #world.set_weather(carla.WeatherParameters.HardRainNoon)
    #world.tick()

    # Get the spectator
    # the transform guide everything. The location has x,y,z.
    # x is front/back, y is left/right, z is up/down
    spectator:carla.Actor = world.get_spectator()

    total_frames = 0
    try:

        anomalies = []
        all_vehicles = []
        sensors = []
        ego_vehicle = spawn_ego_vehicle(world, client, args)
        if ego_vehicle is None:
            raise Exception("Ego vehicle not spawned, exiting...")

        all_vehicles.append(ego_vehicle.id)

        # vehicles is an array of all the ID of the vehicles spawned
        vehicles = generate_traffic(args, world, client)
        all_vehicles.extend(vehicles)

        # The array is composed as follows: [controller, walker, controller, walker, ...]
        contr_walk_array = generate_pedestrian(args, world, client)



        wp:carla.Waypoint = world.get_map().get_waypoint(ego_vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        #3.5 is the width of one lane, so 7 if double lane
        lane_width = wp.lane_width
        if wp.lane_change is not carla.LaneChange.NONE:
            lane_width = lane_width * 2
        # We use the lane_width in order to base the spawn of the anomalies. If lane is smaller, less anomalies
        # are spawned, if lane is bigger, more anomalies are spawned
        if lane_width == 3.5:
            args.anomalies = random.sample(args.anomalies, min(random.randint(1,4),len(args.anomalies)))
        else:
            args.anomalies = random.sample(args.anomalies, min(random.randint(1,6),len(args.anomalies)))
        info("Anomalies to spawn: " + str(args.anomalies))


        # Spawn the anomalies
        if args.anomalies:
            for str_anomaly in args.anomalies:
                anomaly_name = str_anomaly[0]
                size = str_anomaly[1]
                anomaly_object = generate_anomaly_object(world, client, ego_vehicle, anomaly_name, size)
                info("Spawning anomaly: " + str_anomaly[0] + " of size " + str(size))
                anomaly: carla.Actor = anomaly_object.spawn_anomaly()
                if anomaly is None:
                    warning(str_anomaly[0] + " -> Tried to spawn anomaly, but it was not spawned. Skipping it...")
                if anomaly is not None:
                    info(f"Anomaly {anomaly_name} spawned")
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

        #Wait 1.5 second (1.5/simulation_time_for_tick), to let everything spawn properly and adjust their rotation,
        # then set the autopilot to true and the sensors
        for _ in range(int(1.5/simulation_time_for_tick)):
            world.tick()


        #set autopilot for the vehicles and start the simulation
        vehicles_in_world = world.get_actors().filter("*vehicle*")
        anomaly_instances = [anomaly_obj.anomaly.id for anomaly_obj in anomalies]
        for vehicle in vehicles_in_world:
            #There's also the flipped car anomaly that spawn a vehicle, we don't what to set autopilot to it
            if not vehicle.id in anomaly_instances:
                vehicle.set_autopilot(True)

        # Set up the sensors
        set_up_sensors(args, client, ego_vehicle, sensors, world)


        #With a for, it runs N times the simulation
        # 0.05 is the simulation run for each tick.
        loops = args.run_time / simulation_time_for_tick
        for i in range(int(loops)):
            world.tick()
            total_frames += 1
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
                            info(f"Ego vehicle collided with {anomaly_obj.anomaly}, stopping simulation...")
                            flag = True
                            break
                        # If the anomaly is just "attached" to the parent actor, check if the parent actor is the one collided with
                        if anomaly_obj.anomaly.get_parent() is not None:
                            if coll_event.other_actor.id == anomaly_obj.anomaly.get_parent().id:
                                info(f"Ego vehicle collided with {anomaly_obj.anomaly}, stopping simulation...")
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
                    info(f"Ego vehicle passed the furthest anomaly {more_distance_anomaly}, stopping simulation...")
                    break
                for anomaly_obj in anomalies:
                    anomaly_obj.handle_semantic_tag()

            # Handle the sensors data
            handle_sensor_data(args, args.seed, sensors)
            info("Frame: " + str(world.get_snapshot().frame))

    except KeyboardInterrupt:
        error("Simulation interrupted by user.", exc_info=True)
    except Exception as e:
        error(f"An error occurred: {e}", exc_info=True)
        error("Simulation failed. Interrupting...", exc_info=True)
    finally:
        # Handle the anomalies
        if args.anomalies:
            for anomaly_obj in anomalies:
                anomaly_obj.on_destroy()
        clean_up(args,world,client,tm,sensors)
        info("Total frames: " + str(total_frames))
        info("End of simulation")

def handle_sensor_data(args, run, sensors):
    # if the queue is empty, skip one and go to the next tick(). This happens at the first tick() because
    # the data will be available only at the next tick()
    for sensor in sensors:
        try:
            sensor.handle(run,args.anomalies)
        except Empty as e:
            print("Some sensor queue is empty, skipping this tick")

def set_up_sensors(args, client, ego_vehicle, sensors, world):
    if args.rgb:
        info("Setting up RGB camera")
        rgb_sen: carla.Sensor = attach_rbgcamera(args, world, client, ego_vehicle)
        rgb_queue = Queue()
        rgb_sen.listen(lambda data: rgb_queue.put(data))
        sensors.append(RGB_Sensor(rgb_queue,rgb_sen))
    if args.lidar:
        info("Setting up LIDAR sensor")
        lidar_sen: carla.Sensor = attach_lidar(args, world, client, ego_vehicle)
        lidar_queue = Queue()
        lidar_sen.listen(lambda data: lidar_queue.put(data))
        sensors.append(Lidar_Sensor(lidar_queue,lidar_sen))
    if args.semantic:
        info("Setting up Semantic Segmentation camera")
        semantic_sen: carla.Sensor = attach_semantic_camera(args, world, client, ego_vehicle)
        semantic_queue = Queue()
        semantic_sen.listen(lambda data: semantic_queue.put(data))
        sensors.append(Semantic_Sensor(semantic_queue,semantic_sen))
    if args.lidar_semantic:
        info("Setting up Semantic LIDAR sensor")
        lidar_semantic: carla.Sensor = attach_semantic_lidar(args, world, client, ego_vehicle)
        lidar_semantic_queue = Queue()
        lidar_semantic.listen(lambda data: lidar_semantic_queue.put(data))
        sensors.append(Semantic_Lidar_Sensor(lidar_semantic_queue,lidar_semantic))
    if args.radar:
        info("Setting up Radar sensor")
        radar_sen: carla.Sensor = attach_radar(args, world, client, ego_vehicle)
        radar_queue = Queue()
        radar_sen.listen(lambda data: radar_queue.put(data))
        sensors.append(Radar_Sensor(radar_queue,radar_sen))
    if args.depth:
        info("Setting up Depth camera")
        depth_sen: carla.Sensor = attach_depth_camera(args, world, client, ego_vehicle)
        depth_queue = Queue()
        depth_sen.listen(lambda data: depth_queue.put(data))
        sensors.append(Depth_Sensor(depth_queue,depth_sen))
    if args.instance:
        info("Setting up Instance Segmentation camera")
        instance_sen: carla.Sensor = attach_instance_camera(args, world, client, ego_vehicle)
        instance_queue = Queue()
        instance_sen.listen(lambda data: instance_queue.put(data))
        sensors.append(Instant_Segmentation_Sensor(instance_queue,instance_sen))

def generate_anomaly_object(world, client, ego_vehicle, name, size):
    if name == "labrador":
        return Labrador_Anomaly(world, client, name, ego_vehicle,size)
    if name == "baseballbat":
        return Baseballbat_Anomaly(world, client, name, ego_vehicle,size)
    if name == "basketball":
        return Basketball_Anomaly(world, client, name, ego_vehicle,size)
    if name == "person":
        return Person_Anomaly(world, client, name, ego_vehicle,size)
    if name == "tree":
        return Tree_Anomaly(world, client, name, ego_vehicle,size)
    if name == "beerbottle":
        return Beer_Anomaly(world, client, name, ego_vehicle,size)
    if name == "football":
        return Football_Anomaly(world, client, name, ego_vehicle,size)
    if name == "ladder":
        return Ladder_Anomaly(world, client, name, ego_vehicle,size)
    if name == "mattress":
        return Mattress_Anomaly(world, client, name, ego_vehicle,size)
    if name == "skateboard":
        return Skateboard_Anomaly(world, client, name, ego_vehicle,size)
    if name == "tire":
        return Tire_Anomaly(world, client, name, ego_vehicle,size)
    if name == "woodpalette":
        return WoodPalette_Anomaly(world, client, name, ego_vehicle,size)
    if name == "basketball_bounce":
        return Basketball_Bounce_Anomaly(world, client, name, ego_vehicle,size)
    if name == "football_bounce":
        return Football_Bounce_Anomaly(world, client, name, ego_vehicle,size)
    if name == "streetlight":
        return StreetLight_Anomaly(world, client, name, ego_vehicle,size)
    if name == "trashcan":
        return TrashCan_Anomaly(world, client, name, ego_vehicle,size)
    if name == "trafficlight":
        return TrafficLight_Anomaly(world, client, name, ego_vehicle,size)
    if name == "flippedcar":
        return FlippedCar_Anomaly(world, client, name, ego_vehicle,size)
    if name == "instantcarbreak":
        return InstantCarBreak_Anomaly(world, client, name, ego_vehicle,size)
    if name == "trafficlightoff":
        return TrafficLightOff_Anomaly(world, client, name, ego_vehicle,size)
    if name == "carthroughredlight":
        return CarThroughRedLight_Anomaly(world, client, name, ego_vehicle,size)
    if name == "roadsigntwisted":
        return RoadSignTwisted_Anomaly(world, client, name, ego_vehicle,size)
    if name == "roadsignvandalized":
        return RoadSignVandalized_Anomaly(world, client, name, ego_vehicle,size)
    # if name == "motorcycle":
    #     return Motorcycle_Anomaly(world, client, name, ego_vehicle,size)
    if name == "garbagebag":
        return GarbageBag_Anomaly(world, client, name, ego_vehicle,size)
    if name == "garbagebagwind":
        return GarbageBagWind_Anomaly(world, client, name, ego_vehicle,size)
    if name == "brokenchair":
        return BrokenChair_Anomaly(world, client, name, ego_vehicle,size)
    if name == "bikes":
        return Bikes_Anomaly(world, client, name, ego_vehicle,size)
    if name == "hubcap":
        return Hubcap_Anomaly(world, client, name, ego_vehicle,size)
    if name == "newspaper":
        return Newspaper_Anomaly(world, client, name, ego_vehicle,size)
    if name == "blowingnewspaper":
        return BlowingNewspaper_Anomaly(world, client, name, ego_vehicle,size)
    if name == "box":
        return Box_Anomaly(world, client, name, ego_vehicle,size)
    if name == "plasticbottle":
        return PlasticBottle_Anomaly(world, client, name, ego_vehicle,size)
    if name == "winebottle":
        return WineBottle_Anomaly(world, client, name, ego_vehicle,size)
    if name == "metalbottle":
        return MetalBottle_Anomaly(world, client, name, ego_vehicle,size)
    if name == "table":
        return Table_Anomaly(world, client, name, ego_vehicle,size)
    if name == "officechair":
        return OfficeChair_Anomaly(world, client, name, ego_vehicle,size)
    if name == "oldstove":
        return OldStove_Anomaly(world, client, name, ego_vehicle,size)
    if name == "shoppingcart":
        return ShoppingCart_Anomaly(world, client, name, ego_vehicle,size)
    if name == "bag":
        return Bag_Anomaly(world, client, name, ego_vehicle,size)
    if name == "helmet":
        return Helmet_Anomaly(world, client, name, ego_vehicle,size)
    if name == "hat":
        return Hat_Anomaly(world, client, name, ego_vehicle,size)
    if name == "crash":
        return Crash_Anomaly(world, client, name, ego_vehicle,size)
    if name == "bird":
        return Bird_Anomaly(world, client, name, ego_vehicle,size)
    if name == "trafficcone":
        return TrafficCone_Anomaly(world, client, name, ego_vehicle,size)
    if name == "dangerdriver":
        return DangerDriver_Anomaly(world, client, name, ego_vehicle,size)
    if name == "billboard":
        return BillBoard_Anomaly(world, client, name, ego_vehicle,size)
    if name == "fallenstreetlight":
        return FallenStreetLight_Anomaly(world, client, name, ego_vehicle,size)
    if name == "book":
        return Book_Anomaly(world, client, name, ego_vehicle,size)
    if name == "stroller":
        return Stroller_Anomaly(world, client, name, ego_vehicle,size)
    if name == "fuelcan":
        return FuelCan_Anomaly(world, client, name, ego_vehicle,size)
    if name == "constructionbarrier":
        return ConstructionBarrier_Anomaly(world, client, name, ego_vehicle,size)
    # if name == "constructionsite":
    #     return ConstructionSite_Anomaly(world, client, name, ego_vehicle,size)
    if name == "suitcase":
        return Suitcase_Anomaly(world, client, name, ego_vehicle,size)
    if name == "carmirror":
        return CarMirror_Anomaly(world, client, name, ego_vehicle,size)
    if name == "drone":
        return Drone_Anomaly(world, client, name, ego_vehicle,size)
    if name == "umbrella":
        return Umbrella_Anomaly(world, client, name, ego_vehicle,size)
    if name == "tierscooter":
        return TierScooter_Anomaly(world, client, name, ego_vehicle,size)
    if name == "scooter":
        return Scooter_Anomaly(world, client, name, ego_vehicle,size)
    if name == "brick":
        return Brick_Anomaly(world, client, name, ego_vehicle,size)
    if name == "cardoor":
        return CarDoor_Anomaly(world, client, name, ego_vehicle,size)
    if name == "rock":
        return Rock_Anomaly(world, client, name, ego_vehicle,size)
    if name == "hoodcar":
        return HoodCar_Anomaly(world, client, name, ego_vehicle,size)
    if name == "trunkcar":
        return TrunkCar_Anomaly(world, client, name, ego_vehicle,size)
    if name == "kidtoy":
        return KidToy_Anomaly(world, client, name, ego_vehicle,size)
    if name == "mannequin":
        return Mannequin_Anomaly(world, client, name, ego_vehicle,size)
    if name == "tablet":
        return Tablet_Anomaly(world, client, name, ego_vehicle,size)
    if name == "laptop":
        return Laptop_Anomaly(world, client, name, ego_vehicle,size)
    if name == "smartphone":
        return Smartphone_Anomaly(world, client, name, ego_vehicle,size)
    if name == "television":
        return Television_Anomaly(world, client, name, ego_vehicle,size)
    if name == "washingmachine":
        return WashingMachine_Anomaly(world, client, name, ego_vehicle,size)
    if name == "fridge":
        return Fridge_Anomaly(world, client, name, ego_vehicle,size)
    if name == "pilesand":
        return PileSand_Anomaly(world, client, name, ego_vehicle,size)
    if name == "shovel":
        return Shovel_Anomaly(world, client, name, ego_vehicle,size)
    if name == "rake":
        return Rake_Anomaly(world, client, name, ego_vehicle,size)
    if name == "deliverybox":
        return DeliveryBox_Anomaly(world, client, name, ego_vehicle,size)
    if name == "fallentree":
        return FallenTree_Anomaly(world, client, name, ego_vehicle,size)
    if name == "oven":
        return Oven_Anomaly(world, client, name, ego_vehicle,size)
    if name == "wheelchair":
        return WheelChair_Anomaly(world, client, name, ego_vehicle,size)
    if name == "shoe":
        return Shoe_Anomaly(world, client, name, ego_vehicle,size)
    if name == "glove":
        return Glove_Anomaly(world, client, name, ego_vehicle,size)
    if name == "hammer":
        return Hammer_Anomaly(world, client, name, ego_vehicle,size)
    if name == "wrench":
        return Wrench_Anomaly(world, client, name, ego_vehicle,size)
    if name == "drill":
        return Drill_Anomaly(world, client, name, ego_vehicle,size)
    if name == "saw":
        return Saw_Anomaly(world, client, name, ego_vehicle,size)
    if name == "sunglasses":
        return Sunglasses_Anomaly(world, client, name, ego_vehicle,size)
    if name == "wallet":
        return Wallet_Anomaly(world, client, name, ego_vehicle,size)
    if name == "coffecup":
        return CoffeeCup_Anomaly(world, client, name, ego_vehicle,size)
    if name == "fence":
        return Fence_Anomaly(world, client, name, ego_vehicle,size)
    if name == "pizzabox":
        return PizzaBox_Anomaly(world, client, name, ego_vehicle,size)
    if name == "toycar":
        return ToyCar_Anomaly(world, client, name, ego_vehicle,size)
    if name == "remotecontrol":
        return RemoteControl_Anomaly(world, client, name, ego_vehicle,size)
    if name == "cd":
        return CD_Anomaly(world, client, name, ego_vehicle,size)
    if name == "powerbank":
        return PowerBank_Anomaly(world, client, name, ego_vehicle,size)
    if name == "deodorant":
        return Deodorant_Anomaly(world, client, name, ego_vehicle,size)
    if name == "lighter":
        return Lighter_Anomaly(world, client, name, ego_vehicle,size)
    if name == "bowl":
        return Bowl_Anomaly(world, client, name, ego_vehicle,size)
    if name == "bucket":
        return Bucket_Anomaly(world, client, name, ego_vehicle,size)
    if name == "speaker":
        return Speaker_Anomaly(world, client, name, ego_vehicle,size)
    if name == "guitar":
        return Guitar_Anomaly(world, client, name, ego_vehicle,size)
    if name == "pillow":
        return Pillow_Anomaly(world, client, name, ego_vehicle,size)
    if name == "fan":
        return Fan_Anomaly(world, client, name, ego_vehicle,size)
    if name == "dumbell":
        return Dumbell_Anomaly(world, client, name, ego_vehicle,size)
    if name == "trolley":
        return Trolley_Anomaly(world, client, name, ego_vehicle,size)
    warning("Anomaly " + name + " not found, returning None")
    return None

def build_arg_anomalies(list,size):
    anomalies = []
    for name in list:
        anomalies.append([name,size])
    return anomalies

def benchmark():
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
    parser.add_argument('--sensor_tick', type=str, default='0.1',
                        help='Sensor tick time in seconds (This is the FPS of the sensors)')
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
    parser.add_argument("--spawn_points", nargs="+", type=carla.Transform, help="List of spawn_points for ego vehicle", default=None)
    args = parser.parse_args()

    list_static_anomalies = [
        "baseballbat", "basketball", "beerbottle", "football", "ladder", "mattress", "skateboard", "tire", "woodpalette",
        "roadsigntwisted", "roadsignvandalized",
        "garbagebag", "brokenchair", "hubcap",
        "newspaper", "box", "plasticbottle", "winebottle", "metalbottle", "table", "officechair", "oldstove", "shoppingcart",
        "bag", "helmet", "hat", "trafficcone", "fallenstreetlight", "book", "stroller", "fuelcan", "constructionbarrier",

        # TO TEST sem seg
        "suitcase", "carmirror", "umbrella", "tierscooter", "brick", "cardoor", "rock", "hoodcar", "trunkcar", "kidtoy", "mannequin", "tablet",
        "laptop", "smartphone", "television", "scooter",
        "washingmachine", "fridge", "pilesand", "shovel", "rake", "deliverybox", "fallentree", "oven",
        "wheelchair", "hammer", "wrench", "shoe", "glove",
        "drill", "saw", "sunglasses", "bikes", "flippedcar",
        "wallet", "coffecup", "fence", "pizzabox", "toycar", "remotecontrol", "cd", "powerbank", "deodorant", "lighter",
        "bowl", "bucket", "speaker", "guitar", "pillow", "fan", "trolley", "dumbell"
    ]

    list_tiny_anomalies = [
        "beerbottle", "cd", "coffecup", "deodorant", "dumbell", "lighter", "plasticbottle",
        "powerbank", "remotecontrol", "saw", "smartphone", "wallet", "wrench", "hammer", "sunglasses"
    ]

    list_small_anomalies = [
        "baseballbat", "book", "metalbottle", "bowl", "brick", "bucket", "deliverybox",
        "drill", "fan", "football", "fuelcan", "glove", "guitar", "hubcap", "kidtoy",
        "laptop", "newspaper", "pizzabox", "rock", "shoe", "shovel",
        "tablet", "winebottle", "suitcase", "carmirror", "umbrella",
        "speaker",
        #DYN
        "blowingnewspaper", "football_bounce", "basketball_bounce"
    ]

    list_medium_anomalies = [
        "bag", "basketball", "bikes", "brokenchair", "cardoor", "constructionbarrier",
        "fence", "fridge", "garbagebag", "hat", "helmet", "hoodcar", "ladder",
        "mannequin", "mattress", "officechair", "oldstove", "oven", "pillow",
        "rake", "skateboard", "stroller", "television", "tire", "toycar",
        "trafficcone", "roadsigntwisted", "roadsignvandalized", "trolley",
        "trunkcar", "washingmachine", "woodpalette", "wheelchair", "box",
        #DYN
        "labrador", "person", "bird", "trashcan", "garbagebagwind"
        #"drone"
    ]

    list_large_anomalies = [
        "fallenstreetlight", "fallentree", "flippedcar", "pilesand",
        "scooter", "shoppingcart", "table", "tierscooter",
        #DYN
        "dangerdriver", "crash", "streetlight", "tree", "trafficlight", "trafficlightoff",
        "billboard", "instantcarbreak", "carthroughredlight"
    ]

    list_dynamic_anomalies = [
        "labrador", "person", "bird", "dangerdriver", "crash", "drone", "instantcarbreak", "carthroughred",
        "tree", "bouncingbasketball", "bouncingfootball", "streetlight", "trashcan", "trafficlight", "trafficlightoff",
        "garbagebagwind", "newspaperwind", "billboard"
    ]

    print("Total Anomalies: ", len(list_tiny_anomalies) + len(list_small_anomalies) + len(list_medium_anomalies) + len(list_large_anomalies))

    args.semantic = True
    args.rgb = True
    args.depth = True
    args.lidar = True
    args.radar = True
    args.instance = True
    args.lidar_semantic = True

    number_of_runs = 1
    seed = 40
    args.log = True
    # NOTE: fps and sensor_tick are linked. If I have 20 fps, then the tick will be every 1/20 = 0.05 seconds. So the sensor tick should be >= 0.05.
    # If the set sensor_tick 0.1 with an FPS of 20, the sensor will capture data every 2 frames.
    args.fps = 20
    args.sensor_tick = '0.05'
    args.hybrid = True


    runs = {
        0 : {"fps": 20, "sensor_tick": '0.1'},
        # 1 : {"fps": 20, "sensor_tick": '0.05'},
        # 2 : {"fps": 30, "sensor_tick": '0.035'},
        # 3 : {"fps": 40, "sensor_tick": '0.025'},
        # 4:  {"fps": 50, "sensor_tick": '0.02'},
        # 5:  {"fps": 60, "sensor_tick": '0.017'},
    }

    for run in range(number_of_runs):
        random.seed(seed+run)
        args.seed = seed+run

        number_of_tiny_anomalies = random.randint(1, 3)
        tiny_anomalies = random.sample(list_tiny_anomalies, number_of_tiny_anomalies)
        tiny = build_arg_anomalies(tiny_anomalies,"tiny")

        number_of_small_anomalies = random.randint(1, 3)
        small_anomalies = random.sample(list_small_anomalies, number_of_small_anomalies)
        small = build_arg_anomalies(small_anomalies,"small")

        number_of_medium_anomalies = random.randint(0, 2)
        medium_anomalies = random.sample(list_medium_anomalies, number_of_medium_anomalies)
        medium = build_arg_anomalies(medium_anomalies,"medium")

        number_of_large_anomalies = random.randint(0, 2)
        large_anomalies = random.sample(list_large_anomalies, number_of_large_anomalies)
        large = build_arg_anomalies(large_anomalies,"large")


        args.anomalies = tiny + small + medium + large

        #randomly select number of vehicles
        args.number_of_vehicles = random.randint(10, 40)
        #args.number_of_vehicles = 0

        #randomly select number of pedestrians
        args.number_of_pedestrians = random.randint(10, 40)
        #args.number_of_pedestrians = 0
        #args.spawn_points = [carla.Transform(carla.Location(x=-20,y=135.89886719,z=0.6), carla.Rotation(0,0,0))]

        print(f"Running run {run+1} / {number_of_runs} with the following parameters:")
        print(args)

        start_time = time.time()
        main(args,run)
        end_time = time.time()
        print(f"Execution Time: {end_time - start_time} seconds\n\n\n")

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
    parser.add_argument("--anomalies", nargs="+", type=str, help="Dict of anomalies to spawn and size", default=None)
    parser.add_argument("--spawn_points", nargs="+", type=str, help="List of spawn_points for ego vehicle", default=None)
    parser.add_argument("--log", action='store_true', help="Log to file", default=False)

    #args = parser.parse_args()

    #main(args)

    benchmark()