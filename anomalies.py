from utils import *

class Anomaly:
    def __init__(self, world: carla.World, client: carla.Client, name ,ego_vehicle, is_dynamic, anomaly_in_waypoint):
        self.world = world
        self.client = client
        self.ego_vehicle = ego_vehicle
        self.anomaly_in_waypoint = anomaly_in_waypoint
        self.map:carla.Map = self.world.get_map()
        self.name = name
        self.is_dynamic = is_dynamic
        self.anomaly: carla.Actor = None

    def spawn_anomaly(self):
        print("Spawning anomaly...", self.name)
        anomaly = spawn_anomaly(self.world, self.client, self.ego_vehicle, self.name, self.is_dynamic, self.anomaly_in_waypoint)
        self.anomaly = anomaly
        print("Anomaly Spawned!")
        return anomaly

    def handle_semantic_tag(self):
        raise NotImplementedError("This method should be overridden by subclasses")

class Labrador_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle, anomaly_in_waypoint):
        super().__init__(world, client, name, ego_vehicle, True, anomaly_in_waypoint)
        self.distance_from_sidewalk = 2

    def handle_semantic_tag(self):
        current_wp_loc = self.map.get_waypoint(self.anomaly.get_location(), project_to_road=True, lane_type=carla.LaneType.Sidewalk).transform.location
        current_loc = self.anomaly.get_location()
        if current_wp_loc.distance(current_loc) > self.distance_from_sidewalk:
            self.anomaly.set_actor_semantic_tag("Dynamic_Anomaly")

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Baseballbat_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle, anomaly_in_waypoint):
        super().__init__(world, client, name, ego_vehicle, False, anomaly_in_waypoint)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Basketball_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle, anomaly_in_waypoint):
        super().__init__(world, client, name, ego_vehicle, False, anomaly_in_waypoint)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Person_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle, anomaly_in_waypoint):
        super().__init__(world, client, name, ego_vehicle, True, anomaly_in_waypoint)

    def handle_semantic_tag(self):
        current_wp_loc = self.map.get_waypoint(self.anomaly.get_location(), project_to_road=True, lane_type=carla.LaneType.Sidewalk).transform.location
        current_loc = self.anomaly.get_location()
        if current_wp_loc.distance(current_loc) > 2:
            self.anomaly.set_actor_semantic_tag("Dynamic_Anomaly")

    def spawn_anomaly(self):
        return super().spawn_anomaly()

class Tree_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle, anomaly_in_waypoint):
        super().__init__(world, client, name, ego_vehicle, True, anomaly_in_waypoint)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()