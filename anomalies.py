from clean_up import world
from utils import *

class Anomaly:
    def __init__(self, world: carla.World, client: carla.Client, name ,ego_vehicle, is_dynamic, is_character, can_be_rotated, anomaly_in_waypoint):
        self.world = world
        self.client = client
        self.ego_vehicle = ego_vehicle
        self.anomaly_in_waypoint = anomaly_in_waypoint
        self.map:carla.Map = self.world.get_map()
        self.name = name
        self.is_dynamic = is_dynamic
        self.is_character = is_character
        self.can_be_rotated = can_be_rotated
        self.anomaly: carla.Actor = None

    def spawn_anomaly(self):
        print("Spawning anomaly...", self.name)
        anomaly = spawn_anomaly(self.world, self.client, self.ego_vehicle, self.name, self.is_dynamic, self.is_character, self.can_be_rotated, self.anomaly_in_waypoint)
        self.anomaly = anomaly
        if self.anomaly:
            print("Anomaly Spawned!")
        return anomaly

    def handle_semantic_tag(self):
        raise NotImplementedError("This method should be overridden by subclasses")

class Labrador_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, True, True, False, True)
        self.distance_from_sidewalk = 2

    def handle_semantic_tag(self):
        current_wp_loc = self.map.get_waypoint(self.anomaly.get_location(), project_to_road=True, lane_type=carla.LaneType.Sidewalk).transform.location
        current_loc = self.anomaly.get_location()
        if current_wp_loc.distance(current_loc) > self.distance_from_sidewalk:
            self.anomaly.set_actor_semantic_tag("Dynamic_Anomaly")

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Baseballbat_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Basketball_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Person_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, True, True, False, True)

    def handle_semantic_tag(self):
        current_wp_loc = self.map.get_waypoint(self.anomaly.get_location(), project_to_road=True, lane_type=carla.LaneType.Sidewalk).transform.location
        current_loc = self.anomaly.get_location()
        if current_wp_loc.distance(current_loc) > 2:
            self.anomaly.set_actor_semantic_tag("Dynamic_Anomaly")

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Tree_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, True, False, False, True)

    def handle_semantic_tag(self):
        #If the tree is starting to fall, we want to set the semantic tag to Dynamic_Anomaly
        current_rotation = self.anomaly.get_transform().rotation
        # pitch is the x axis rotation, roll is the y axis rotation
        if abs(current_rotation.pitch) > 10 or abs(current_rotation.roll) > 10:
            self.anomaly.set_actor_semantic_tag("Dynamic_Anomaly")


    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Beer_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Football_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Ladder_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False,False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Mattress_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False,False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Skateboard_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Tire_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True,False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class WoodPalette_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False,False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Basketball_Bounce_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Football_Bounce_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class StreetLight_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, True, False, False, True)

    def handle_semantic_tag(self):
        current_rotation = self.anomaly.get_transform().rotation
        # pitch is the x axis rotation, roll is the y axis rotation
        if abs(current_rotation.pitch) > 10 or abs(current_rotation.roll) > 10:
            self.anomaly.set_actor_semantic_tag("Dynamic_Anomaly")

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class TrashCan_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, True, False, False, True)

    def handle_semantic_tag(self):
        current_wp_loc = self.map.get_waypoint(self.anomaly.get_location(), project_to_road=True, lane_type=carla.LaneType.Sidewalk).transform.location
        current_loc = self.anomaly.get_location()
        if current_wp_loc.distance(current_loc) > 3:
            self.anomaly.set_actor_semantic_tag("Dynamic_Anomaly")

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class TrafficLight_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, True, False, False, True)

    def handle_semantic_tag(self):
        current_rotation = self.anomaly.get_transform().rotation
        # pitch is the x axis rotation, roll is the y axis rotation
        if abs(current_rotation.pitch) > 10 or abs(current_rotation.roll) > 10:
            self.anomaly.set_actor_semantic_tag("Dynamic_Anomaly")

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class FlippedCar_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        vehicle = random.choice(self.world.get_blueprint_library().filter("vehicle.*"))
        self.name = vehicle.id
        anomaly = super().spawn_anomaly()
        #make it tick, otherwise the location is 0,0,0 (after the tick is updated). If we don't tick, the car will not be spawned correctly
        self.world.tick()
        loc = anomaly.get_location()
        loc.z += 1.5  # Raise the car slightly above the ground
        rot = anomaly.get_transform().rotation
        # Flip the car
        rot.yaw = random.randint(-180, 180)
        rot.roll = 180
        anomaly.set_transform(carla.Transform(loc,rot))
        self.world.tick()
        # Set the semantic tag to Static_Anomaly
        anomaly.set_actor_semantic_tag("Static_Anomaly")
        #Make the ego vehicle ignore the flipped car (It's going to crash into it)
        tm:carla.TrafficManager = self.client.get_trafficmanager()
        tm.collision_detection(self.ego_vehicle, anomaly, False)
        return anomaly