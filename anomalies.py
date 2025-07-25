import sys
import time

from clean_up import world
from utils import *

class Anomaly:
    def __init__(self, world: carla.World, client: carla.Client, name ,ego_vehicle:carla.Actor, is_dynamic, is_character, can_be_rotated, anomaly_in_waypoint, spawn_at_zero=False):
        self.world = world
        self.client = client
        self.ego_vehicle = ego_vehicle
        self.anomaly_in_waypoint = anomaly_in_waypoint
        self.map:carla.Map = self.world.get_map()
        self.name = name
        self.is_dynamic = is_dynamic
        self.is_character = is_character
        self.can_be_rotated = can_be_rotated
        self.spawn_at_zero = spawn_at_zero
        self.anomaly: carla.Actor = None
        self.tick = 0

    def spawn_anomaly(self):
        print("Spawning anomaly...", self.name)
        anomaly = spawn_anomaly(self.world, self.client, self.ego_vehicle, self.name, self.is_dynamic, self.is_character, self.can_be_rotated, self.anomaly_in_waypoint, self.spawn_at_zero)
        self.anomaly = anomaly
        if self.anomaly:
            print("Anomaly Spawned!")
        return anomaly

    def handle_semantic_tag(self):
        raise NotImplementedError("This method should be overridden by subclasses")

    def find_obj_in_front_ego_vehicle(self, filter_string: str, min_distance: float = 10, max_distance: float = 20, angle: float = 0.95):
        actors: carla.ActorList = self.world.get_actors()
        actors_filtered: carla.ActorList = actors.filter(filter_string)
        ego_fw = self.ego_vehicle.get_transform().get_forward_vector()
        ego_loc = self.ego_vehicle.get_transform().location
        parent = None
        for actor in actors_filtered:
            if actor.id != self.ego_vehicle.id:
                dst: carla.Location = actor.get_transform().location - ego_loc
                # We have the distance vector and the forward vector of the ego vehicle. The dot product will tell us if the vehicle is in front of the ego vehicle
                # 0.95 is something like 15°
                if ego_fw.dot(dst.make_unit_vector()) > angle:
                    # If the vehicle is in front of the ego vehicle, we want to check the distance, say between 10 and 20 meters
                    if min_distance < dst.length() < max_distance:
                        # We want to attach the anomaly to this vehicle
                        parent = actor
                        break
        return parent

    def on_destroy(self):
        print(self.name + " -> Destroying anomaly...")

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

    def on_destroy(self):
        super().on_destroy()

class Baseballbat_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

    def on_destroy(self):
        super().on_destroy()

class Basketball_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

    def on_destroy(self):
        super().on_destroy()

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

    def on_destroy(self):
        super().on_destroy()

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

    def on_destroy(self):
        super().on_destroy()

class Beer_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

    def on_destroy(self):
        super().on_destroy()

class Football_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

    def on_destroy(self):
        super().on_destroy()

class Ladder_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False,False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

    def on_destroy(self):
        super().on_destroy()

class Mattress_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False,False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

    def on_destroy(self):
        super().on_destroy()

class Skateboard_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

    def on_destroy(self):
        super().on_destroy()

class Tire_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True,False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

    def on_destroy(self):
        super().on_destroy()

class WoodPalette_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False,False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

    def on_destroy(self):
        super().on_destroy()

class Basketball_Bounce_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

    def on_destroy(self):
        super().on_destroy()

class Football_Bounce_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        return super().spawn_anomaly()

    def on_destroy(self):
        super().on_destroy()

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

    def on_destroy(self):
        super().on_destroy()

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

    def on_destroy(self):
        super().on_destroy()

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

    def on_destroy(self):
        super().on_destroy()

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

    def on_destroy(self):
        super().on_destroy()

class InstantCarBreak_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        # Spawn this at zero, then if in front of the ego vehicle there's a car, it will be attached to this car, so it'll have it's location
        # (This is done at the blueprint level)
        self.parent = None
        self.tm:carla.TrafficManager = client.get_trafficmanager()
        self.first_run = True
        super().__init__(world, client, name, ego_vehicle, False, False, False, False, spawn_at_zero=True)

    def handle_semantic_tag(self):
        # Make the vehicle break after 40 ticks
        if self.first_run and self.tick>=40:
            self.parent.set_autopilot(False)
            self.parent.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1, hand_brake=True))
            self.parent.set_actor_semantic_tag("Static_Anomaly")
            self.first_run = False

    def spawn_anomaly(self):
        print("InstantCarBreak -> Spawning InstantCarBreak anomaly...")
        vehicle_to_attach = self.find_obj_in_front_ego_vehicle("vehicle.*", min_distance=10, max_distance=20)
        if vehicle_to_attach is None:
            print("InstantCarBreak -> No vehicle found in front of the ego vehicle to attach the anomaly to.")
            return None
        else:
            print("InstantCarBreak -> Found vehicle to attach the anomaly to:", vehicle_to_attach)
        bp_lib: carla.BlueprintLibrary = world.get_blueprint_library()
        anomaly_to_spawn = bp_lib.filter(f"*{self.name}")[0]
        transform = carla.Transform(carla.Location(0, 0, 0), carla.Rotation(0, 0, 0))
        self.anomaly:carla.Actor = world.spawn_actor(anomaly_to_spawn, transform, attach_to=vehicle_to_attach, attachment_type=carla.AttachmentType.Rigid)
        if self.anomaly is None:
            print("InstantCarBreak -> Failed to spawn anomaly:", self.name)
            return None
        self.parent = self.anomaly.parent
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class TrafficLightOff_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        self.first_run = True
        super().__init__(world, client, name, ego_vehicle, False, False, False, False, spawn_at_zero=True)

    def handle_semantic_tag(self):
        # If the traffic light is off, we want to set the semantic tag to Static_Anomaly
        print(self.ego_vehicle.get_traffic_light_state())
        if self.first_run and self.tick>=20:
            parent: carla.TrafficLight = self.anomaly.parent
            parent.set_state(carla.TrafficLightState.Off)
            self.anomaly.parent.freeze(True)
            parent.set_actor_semantic_tag("Static_Anomaly")
            self.first_run = False

    def spawn_anomaly(self):
        print("TrafficLightOff -> Spawning TrafficLightOff anomaly...")
        traffic_light = self.find_obj_in_front_ego_vehicle("traffic.traffic_light", min_distance=10, max_distance=50, angle=0.99)
        if traffic_light is None:
            print("TrafficLightOff -> No traffic light found in front of the ego vehicle to attach the anomaly to.")
            return None
        else:
            print("TrafficLightOff -> Found traffic light to attach the anomaly to:", traffic_light)
        bp_lib = self.world.get_blueprint_library()
        anomaly_to_spawn = bp_lib.filter(f"*{self.name}")[0]
        transform = carla.Transform(carla.Location(0, 0, 0), carla.Rotation(0, 0, 0))
        self.anomaly = self.world.spawn_actor(anomaly_to_spawn, transform, attach_to=traffic_light, attachment_type=carla.AttachmentType.Rigid)
        if self.anomaly is None:
            print("TrafficLightOff -> Failed to spawn anomaly:", self.name)
            return None
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()
        self.anomaly.parent.freeze(False)
        self.anomaly.parent.reset_group()
        self.anomaly.parent.retag_actor()