from main import main
import time
import argparse
import random
import carla

def benchmark():
    parser = argparse.ArgumentParser(description='CARLA Simulator')
    parser.add_argument("--seed", type=int, help="Random seed", default=42)
    parser.add_argument("--map", type=str, help="Map name to load", default=None)
    parser.add_argument("--number_of_vehicles", type=int, help="Number of vehicles to spawn", default=10)
    parser.add_argument("--number_of_pedestrians", type=int, help="Number of pedestrians to spawn", default=10)
    parser.add_argument("--filterv", type=str, help="Vehicle filter", default="vehicle.*")
    parser.add_argument("--filterw", type=str, help="Pedestrian filter", default="walker.*")
    parser.add_argument("--hybrid", action='store_true', help="Use hybrid physics mode")
    parser.add_argument("--image_size_x", type=str, help="Image size X", default='1920')
    parser.add_argument("--image_size_y", type=str, help="Image size Y", default='1080')
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
        "bowl", "bucket", "speaker"
    ]

    # fallentree, take the closest tree (make a range to find it)

    print("Total Static Anomalies: ", len(list_static_anomalies))

    list_dynamic_anomalies = [
        "labrador", "person", "bird", "dangerdriver", "crash", "drone", "instantcarbreak", "carthroughred",
        "tree", "bouncingbasketball", "bouncingfootball", "streetlight", "trashcan", "trafficlight", "trafficlightoff",
        "garbagebagwind", "newspaperwind", "billboard"
    ]

    print("Total Dynamic Anomalies: ", len(list_dynamic_anomalies))
    print("Total Anomalies: ", len(list_static_anomalies) + len(list_dynamic_anomalies))

    #args.semantic = True
    #args.rgb = True
    #args.depth = True
    # args.lidar = True
    # args.radar = True
    #args.instance = True
    # args.lidar_semantic = True
    #Modify randomly
    args.number_of_runs = 1
    # Modify randomly
    args.seed = 42

    # NOTE: fps and sensor_tick are linked. If I have 20 fps, then the tick will be every 1/20 = 0.05 seconds. So the sensor tick should be >= 0.05.
    # If the set sensor_tick 0.1 with an FPS of 20, the sensor will capture data every 2 frames.
    args.fps = 20
    args.sensor_tick = '0.1'

    args.hybrid = True


    runs = {
        0 : {"fps": 20, "sensor_tick": '0.1'},
        # 1 : {"fps": 20, "sensor_tick": '0.05'},
        # 2 : {"fps": 30, "sensor_tick": '0.035'},
        # 3 : {"fps": 40, "sensor_tick": '0.025'},
        # 4:  {"fps": 50, "sensor_tick": '0.02'},
        # 5:  {"fps": 60, "sensor_tick": '0.017'},
    }

    # for run in range(args.number_of_runs):
    for run in runs:

        args.fps = runs[run]["fps"]
        args.sensor_tick = runs[run]["sensor_tick"]


        # Random select anomalies (1 to 5)
        number_of_static_anomalies = random.randint(1, 1)
        static_anomalies = random.sample(list_static_anomalies, number_of_static_anomalies)
        args.anomalies = ["fallentree"]

        #randomly select number of vehicles
        args.number_of_vehicles = random.randint(10, 40)
        args.number_of_vehicles = 0

        #randomly select number of pedestrians
        args.number_of_pedestrians = random.randint(10, 40)
        args.number_of_pedestrians = 0

        #args.spawn_points = [carla.Transform(carla.Location(x=14.13009155,y=69.71400391,z=0.6), carla.Rotation(0,0,0))]

        print(f"Running run {run+1} / {len(runs)} with the following parameters:")
        print(args)

        start_time = time.time()
        main(args)
        end_time = time.time()
        print(f"Execution Time: {end_time - start_time} seconds\n\n\n")

if __name__ == "__main__":
    benchmark()
