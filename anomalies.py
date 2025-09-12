import math
import random
import sys
import time
from collections import deque
from typing import Optional

from fontTools.misc.cython import returns
from pandas.io.formats.format import return_docstring

from utils import *

class Anomaly:
    def __init__(self, world: carla.World, client: carla.Client, name ,ego_vehicle:carla.Actor, is_dynamic, is_character, can_be_rotated, anomaly_in_waypoint, spawn_at_zero=False, spawn_on_right=False, big_mesh=False):
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
        self.spawn_on_right = spawn_on_right
        self.big_mesh = big_mesh
        self.anomaly: carla.Actor = None
        self.tick = 0

    def spawn_anomaly(self):
        print("Spawning anomaly...", self.name)
        anomaly = spawn_anomaly(self.world, self.client, self.ego_vehicle, self.name, self.is_dynamic, self.is_character, self.can_be_rotated, self.anomaly_in_waypoint, self.spawn_at_zero, self.spawn_on_right, self.big_mesh)
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

    def find_objs_in_front_ego_vehicle(self, filter_string: str, min_distance: float = 10, max_distance: float = 20, angle: float = 0.95):
        actors: carla.ActorList = self.world.get_actors()
        actors_filtered: carla.ActorList = actors.filter(filter_string)
        ego_fw = self.ego_vehicle.get_transform().get_forward_vector()
        ego_loc = self.ego_vehicle.get_transform().location
        parents = []
        for actor in actors_filtered:
            if actor.id != self.ego_vehicle.id:
                dst: carla.Location = actor.get_transform().location - ego_loc
                # We have the distance vector and the forward vector of the ego vehicle. The dot product will tell us if the vehicle is in front of the ego vehicle
                # 0.95 is something like 15°
                if ego_fw.dot(dst.make_unit_vector()) > angle:
                    # If the vehicle is in front of the ego vehicle, we want to check the distance, say between 10 and 20 meters
                    if min_distance < dst.length() < max_distance:
                        # We want to attach the anomaly to this vehicle
                        parents.append(actor)
        return parents

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
        super().__init__(world, client, name, ego_vehicle, False, False, False,False, spawn_on_right=True)

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
        bp_lib: carla.BlueprintLibrary = self.world.get_blueprint_library()
        anomaly_to_spawn = bp_lib.filter(f"*{self.name}")[0]
        transform = carla.Transform(carla.Location(0, 0, 0), carla.Rotation(0, 0, 0))
        self.anomaly:carla.Actor = self.world.spawn_actor(anomaly_to_spawn, transform, attach_to=vehicle_to_attach, attachment_type=carla.AttachmentType.Rigid)
        if self.anomaly is None:
            print("InstantCarBreak -> Failed to spawn anomaly:", self.name)
            return None
        self.parent = self.anomaly.parent
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class TrafficLightOff_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        self.traffic_light = None
        self.vehicles = None
        super().__init__(world, client, name, ego_vehicle, False, False, False, False, spawn_at_zero=True)

    def handle_semantic_tag(self):
        # If the traffic light is off, we want to set the semantic tag to Static_Anomaly
        if len(self.vehicles) != 0:
            wps = self.traffic_light.get_stop_waypoints()
            for vehicle in self.vehicles:
                if (vehicle.get_transform().location - wps[0].transform.location).dot(wps[0].transform.get_forward_vector()) >3:
                    vehicle.set_actor_semantic_tag("Dynamic_Anomaly")

    def spawn_anomaly(self):
        print("TrafficLightOff -> Spawning TrafficLightOff anomaly...")
        traffic_light = self.find_obj_in_front_ego_vehicle("traffic.traffic_light", min_distance=10, max_distance=50, angle=0.95)
        if traffic_light is None:
            print("TrafficLightOff -> No traffic light found in front of the ego vehicle to attach the anomaly to.")
            return None
        else:
            print("TrafficLightOff -> Found traffic light to attach the anomaly to:", traffic_light)
        traffic_light_group = traffic_light.get_group_traffic_lights()

        # Sort the traffic lights by distance to the ego vehicle. So the first one will be the closest one (i.e. my traffic light)
        # Then make my traffic light green and the others red
        traffic_light_group.sort(key=lambda tl: self.ego_vehicle.get_transform().location.distance(
            tl.get_stop_waypoints()[0].transform.location))
        my_traffic_light = traffic_light_group[0]
        my_traffic_light.set_state(carla.TrafficLightState.Green)
        traffic_light_group.pop(0)  # Remove my traffic light from the group
        for tl in traffic_light_group:
            tl.set_state(carla.TrafficLightState.Red)

        #tick the world to make the traffic light state change effective
        self.world.tick()

        # Now take one of the red traffic light, turn it off and attach the anomaly to it
        self.traffic_light = traffic_light_group[0]
        self.traffic_light.set_state(carla.TrafficLightState.Off)
        self.traffic_light.set_actor_semantic_tag("Static_Anomaly")

        # Now the vehicles are ignoring the traffic light, so they will run as if it was the red light. So take find those vehicles and set them as Dynamic_Anomaly as soon as they start moving
        #in the junction

        wps = self.traffic_light.get_stop_waypoints()
        self.vehicles = self.find_objs_in_front_ego_vehicle("vehicle.*", min_distance=0, max_distance=100, angle=0)

        # Filter the vehicles that are going in the same direction of the waypoint, so the ones that are in front of the traffic light
        self.vehicles = list(
            filter(lambda v: v.get_transform().get_forward_vector().dot(wps[0].transform.get_forward_vector()) > 0.9,
                   self.vehicles))

        # Now in the filtered_vehilces, we may have some vehicles that are spawned after the waypoint, so they already are on the junction, those
        # vehicles has to be ignored.
        # (vehicle.get_transform().location - wps[0].transform.location).dot(wps[0].transform.get_forward_vector()) > 0
        # The difference vehicle.get_transform().location - wps[0].transform.location is a vector from the waypoint to the vehicle
        # Now if this vector has the same direction of the waypoint forward vector, it means the vehicle is after the waypoint, if it has
        # the opposite direction, it means the vehicle is before the waypoint. To compute so we use the dot product.
        # If the dot product is > 0, the vehicle is after the waypoint, so we ignore it
        # We have that 3 meters is a good distance to consider the vehicle before the waypoint
        # This also filter the vehicles that are in the same direction on the traffic light, but that already passed the junction

        self.vehicles = list(filter(lambda v: (v.get_transform().location - wps[0].transform.location).dot(
            wps[0].transform.get_forward_vector()) < 3, self.vehicles))

        # Now we may have some vehicles that are in the same direction as the traffic light, but on another road, so filter them out
        # We can do this by checking the angle between the difference vector (vehicle - waypoint) and the forward vector. If on the same
        # direction the dot will be 0.9..., otherwise will be 0.0...
        self.vehicles = list(filter(lambda v: (v.get_transform().location - wps[0].transform.location).make_unit_vector().dot(
            wps[0].transform.get_forward_vector()*-1) > 0.7, self.vehicles))

        bp_lib = self.world.get_blueprint_library()
        anomaly_to_spawn = bp_lib.filter(f"*{self.name}")[0]
        transform = carla.Transform(carla.Location(0, 0, 0), carla.Rotation(0, 0, 0))
        self.anomaly = self.world.spawn_actor(anomaly_to_spawn, transform, attach_to=self.traffic_light, attachment_type=carla.AttachmentType.Rigid)
        if self.anomaly is None:
            print("TrafficLightOff -> Failed to spawn anomaly:", self.name)
            return None
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()
        self.traffic_light.freeze(False)
        self.traffic_light.reset_group()
        self.traffic_light.retag_actor()

class CarThroughRedLight_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        self.vehicles = []
        self.traffic_light = None
        super().__init__(world, client, name, ego_vehicle, False, False, False, False, spawn_at_zero=True)

    def handle_semantic_tag(self):
        wps = self.traffic_light.get_stop_waypoints()
        if (self.vehicles[0].get_transform().location - wps[0].transform.location).dot(wps[0].transform.get_forward_vector()) > 3:
            self.vehicles[0].set_actor_semantic_tag("Dynamic_Anomaly")

    def spawn_anomaly(self):
        print("CarThroughRedLight -> Spawning CarThroughRedLight anomaly...")
        traffic_light = self.find_obj_in_front_ego_vehicle("traffic.traffic_light", min_distance=10, max_distance=50, angle=0.95)
        if traffic_light is None:
            print("CarThroughRedLight -> No traffic light found in front of the ego vehicle to attach the anomaly to.")
            return None

        print("CarThroughRedLight -> Found traffic light to attach the anomaly to:", traffic_light)
        traffic_light_group = traffic_light.get_group_traffic_lights()

        #Sort the traffic lights by distance to the ego vehicle. So the first one will be the closest one (i.e. my traffic light)
        #Then make my traffic light green and the others red
        traffic_light_group.sort(key=lambda tl: self.ego_vehicle.get_transform().location.distance(tl.get_stop_waypoints()[0].transform.location))
        my_traffic_light = traffic_light_group[0]
        my_traffic_light.set_state(carla.TrafficLightState.Green)
        traffic_light_group.pop(0)  # Remove my traffic light from the group
        for tl in traffic_light_group:
            tl.set_state(carla.TrafficLightState.Red)

        #Now take one of the red traffic light and check the vehicles in front of it
        self.traffic_light = traffic_light_group[0]

        wps = self.traffic_light.get_stop_waypoints()
        vehicles = self.find_objs_in_front_ego_vehicle("vehicle.*", min_distance=0, max_distance=60, angle=0)

        #Filter the vehicles that are going in the same direction of the waypoint, so the ones that are in front of the traffic light
        filtered_vehicles = list(filter(lambda v: v.get_transform().get_forward_vector().dot(wps[0].transform.get_forward_vector()) >0.9, vehicles))


        #Now in the filtered_vehilces, we may have some vehicles that are spawned after the waypoint, so they already are on the junction, those
        # vehicles has to be ignored.
        # (vehicle.get_transform().location - wps[0].transform.location).dot(wps[0].transform.get_forward_vector()) > 0
        # The difference vehicle.get_transform().location - wps[0].transform.location is a vector from the waypoint to the vehicle
        # Now if this vector has the same direction of the waypoint forward vector, it means the vehicle is after the waypoint, if it has
        # the opposite direction, it means the vehicle is before the waypoint. To compute so we use the dot product.
        # If the dot product is > 0, the vehicle is after the waypoint, so we ignore it
        # We have that 3 meters is a good distance to consider the vehicle before the waypoint

        self.vehicles = list(filter(lambda v: (v.get_transform().location - wps[0].transform.location).dot(wps[0].transform.get_forward_vector()) <3, filtered_vehicles))

        self.vehicles = list(
            filter(lambda v: (v.get_transform().location - wps[0].transform.location).make_unit_vector().dot(
                wps[0].transform.get_forward_vector() * -1) > 0.7, self.vehicles))
        # If there's no vehicle in front of the traffic light, we can't make the anomaly happen, so close the execution
        if len(self.vehicles) == 0:
            print("CarThroughRedLight -> No vehicle found in front of the traffic light in order to make the anomaly happen.")
            return None

        #Now we sort the vehicles by distance to the waypoint, so the first one is the closest to the waypoint
        self.vehicles.sort(key=lambda v: v.get_transform().location.distance(wps[0].transform.location))

        # We take the first vehicle, make it ignore the traffic light and attach the anomaly to it
        tm = self.client.get_trafficmanager()
        tm.ignore_lights_percentage(self.vehicles[0], 100)

        bp_lib = self.world.get_blueprint_library()
        anomaly_to_spawn = bp_lib.filter(f"*{self.name}")[0]
        transform = carla.Transform(carla.Location(0, 0, 0), carla.Rotation(0, 0, 0))
        self.anomaly = self.world.spawn_actor(anomaly_to_spawn, transform, attach_to=self.vehicles[0], attachment_type=carla.AttachmentType.Rigid)
        if self.anomaly is None:
            print("CarThroughRedLight -> Failed to spawn anomaly:", self.name)
            return None

        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class RoadSignTwisted_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        self.old_trf = None
        self.sign = None
        super().__init__(world, client, name, ego_vehicle, True, False, False, True)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        sign = self.find_objs_in_front_ego_vehicle("traffic.*", min_distance=10, max_distance=30, angle=0.90)
        # The filter "traffic.*" also returns traffic lights, so we need to filter them out
        sign = list(filter(lambda a: a.type_id != "traffic.traffic_light", sign))
        if len(sign) == 0:
            print("RoadSignTwisted -> No traffic sign found in front of the ego vehicle to attach the anomaly to.")
            return None
        self.sign = sign[0]

        bp_lip = self.world.get_blueprint_library()
        anomaly_to_spawn = bp_lip.filter(f"*{self.name}")[0]
        transform = carla.Transform(carla.Location(0, 0, 0), carla.Rotation(0, 0, 0))
        self.anomaly: carla.Actor = self.world.spawn_actor(anomaly_to_spawn, transform, attach_to=self.sign)
        self.sign.set_actor_semantic_tag("static_anomaly")
        if self.anomaly is None:
            print("RoadSignTwisted -> Failed to spawn anomaly:", self.name)
            return None
        return self.anomaly

    def on_destroy(self):
        self.sign.retag_actor()
        super().on_destroy()

class RoadSignVandalized_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        self.possible_anomalies = ["stopdestroyed","stoprusted"]
        self.old_trf = None
        self.sign = None
        super().__init__(world, client, name, ego_vehicle, True, False, False, True)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        sign = self.find_objs_in_front_ego_vehicle("traffic.*", min_distance=10, max_distance=30, angle=0.90)
        # The filter "traffic.*" also returns traffic lights, so we need to filter them out
        sign = list(filter(lambda a: a.type_id != "traffic.traffic_light", sign))
        if len(sign) == 0:
            print("RoadSignVandalized -> No traffic sign found in front of the ego vehicle to attach the anomaly to.")
            return None
        self.sign = sign[0]
        self.old_trf = self.sign.get_transform()

        bp_lip = self.world.get_blueprint_library()
        anomaly_to_spawn = bp_lip.filter(f"*{self.name}")[0]
        transform = carla.Transform(carla.Location(0, 0, 0), carla.Rotation(0, 0, 0))
        self.anomaly: carla.Actor = self.world.spawn_actor(anomaly_to_spawn, transform, attach_to=self.sign)

        if self.anomaly is None:
            print("RoadSignVandalized -> Failed to spawn anomaly:", self.name)
            return None

        anomaly_to_spawn = bp_lip.filter(random.choice(self.possible_anomalies))[0]
        self.anomaly: carla.Actor = self.world.spawn_actor(anomaly_to_spawn, self.old_trf)

        if self.anomaly is None:
            print("RoadSignVandalized -> Failed to spawn vandalized anomaly:", anomaly_to_spawn)
            return None

        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class Motorcycle_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, True, False, False, True)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        ego_vehicle_location = self.ego_vehicle.get_location()
        wp = self.map.get_waypoint(ego_vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)
        bp_lib = self.world.get_blueprint_library()
        vespa = bp_lib.filter("*vespa*")[0]
        sp = wp.next(16)[0].transform
        sp.location.z = 0.5  # Raise the motorcycle slightly above the ground
        self.anomaly : carla.Vehicle = self.world.spawn_actor(vespa, sp)
        #self.anomaly.set_autopilot(True)
        self.world.tick()
        print(self.anomaly.get_location())
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class GarbageBag_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False,spawn_on_right=True)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class GarbageBagWind_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        self.base_wind = None
        self.right = None
        super().__init__(world, client, name, ego_vehicle, False, False, True, False, spawn_on_right=True)

    def handle_semantic_tag(self):
        front_back = 1
        if random.random() < 0.2:
            front_back = -1
        base_wind = self.base_wind * front_back
        theta = random.uniform(-math.radians(90), math.radians(90))
        dir_x = base_wind.x * math.cos(theta) + self.right.x * math.sin(theta)
        dir_y = base_wind.y * math.cos(theta) + self.right.y * math.sin(theta)
        vertical = random.uniform(-1.5, 2.5)

        force = carla.Vector3D(dir_x, dir_y, vertical)

        torque = carla.Vector3D(random.uniform(-50, 50),
                                random.uniform(-50, 50),
                                random.uniform(-20, 20))

        self.anomaly.add_torque(torque * 5)
        self.anomaly.add_force(force * random.uniform(5, 8))

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.world.tick()
        self.base_wind = random.choice([self.ego_vehicle.get_transform().get_forward_vector(),
                                       self.ego_vehicle.get_transform().get_right_vector() * -1])
        self.right = self.ego_vehicle.get_transform().get_right_vector()
        # In case the base_wind is the right vector, we want to make sure that the right vector is not the same as the base_wind
        if self.base_wind.dot(self.right) != 0:
            self.right = self.ego_vehicle.get_transform().get_forward_vector()
        self.anomaly.add_impulse(self.base_wind*5)
        torque = carla.Vector3D(random.uniform(-50, 50),
                                random.uniform(-50, 50),
                                random.uniform(-20, 20))

        self.anomaly.add_torque(torque * 5)
        self.anomaly.set_actor_semantic_tag("dynamic_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class BrokenChair_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()


class Bikes_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False,spawn_on_right=True)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class Hubcap_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.world.tick()
        trf = self.anomaly.get_transform()
        trf.location.z += 0.2
        trf.rotation.roll = -90
        self.anomaly.set_transform(trf)
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class Newspaper_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class BlowingNewspaper_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        self.start_tick = 0
        self.base_wind = None
        self.right = None
        super().__init__(world, client, name, ego_vehicle, False, False, False, False)

    def handle_semantic_tag(self):
        front_back = 1
        if random.random() < 0.2:
            front_back = -1
        base_wind = self.base_wind * front_back
        theta = random.uniform(-math.radians(90),math.radians(90))
        dir_x = base_wind.x * math.cos(theta) + self.right.x * math.sin(theta)
        dir_y = base_wind.y * math.cos(theta) + self.right.y * math.sin(theta)
        vertical = random.uniform(-1.5,2.5)

        force = carla.Vector3D(dir_x, dir_y, vertical)

        torque = carla.Vector3D(random.uniform(-50, 50),
                                random.uniform(-50, 50),
                                random.uniform(-20, 20))

        self.anomaly.add_torque(torque*5)
        self.anomaly.add_force(force*random.uniform(0,5))

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.start_tick = self.tick
        self.world.tick()
        self.base_wind = self.ego_vehicle.get_transform().get_forward_vector()
        self.right = self.ego_vehicle.get_transform().get_right_vector()
        init_force = carla.Vector3D(self.base_wind.x, self.base_wind.y, self.base_wind.z+0.5).make_unit_vector()
        self.anomaly.add_impulse(init_force)
        self.anomaly.set_actor_semantic_tag("dynamic_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class Box_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False, spawn_on_right=True)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class PlasticBottle_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False, spawn_on_right=True)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class WineBottle_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False, spawn_on_right=True)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class MetalBottle_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False, spawn_on_right=True)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class Table_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False, spawn_on_right=True, big_mesh=True)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.world.tick()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

class OfficeChair_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False, spawn_on_right=True, big_mesh=True)
    def handle_semantic_tag(self):
        pass
    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly
    def on_destroy(self):
        super().on_destroy()

class OldStove_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False, spawn_on_right=True, big_mesh=True)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class ShoppingCart_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False, spawn_on_right=True, big_mesh=True)
    def handle_semantic_tag(self):
        pass
    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.world.tick()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly
    def on_destroy(self):
        super().on_destroy()

class Bag_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False, spawn_on_right=False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class Helmet_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False, spawn_on_right=False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class Hat_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, True, False, spawn_on_right=False)

    def handle_semantic_tag(self):
        pass

    def spawn_anomaly(self):
        self.anomaly = super().spawn_anomaly()
        self.anomaly.set_actor_semantic_tag("static_anomaly")
        return self.anomaly

    def on_destroy(self):
        super().on_destroy()

class Crash_Anomaly(Anomaly):
    def __init__(self, world: carla.World, client: carla.Client,name: str, ego_vehicle):
        super().__init__(world, client, name, ego_vehicle, False, False, False, False)
        self.target_v = None
        self.coll_queue = None
        self.coll_sen = None
        self.other_v = None
        self.wp_target_v = None
        self.wp_other_v = None
        self.tm:carla.TrafficManager = None
        self.target_tick = random.randint(30,50)
        self.world.debug.draw_arrow(carla.Location(0,0,0), self.ego_vehicle.get_location(), 0.1, 0.1, carla.Color(255,0,0), 5)

    def spawn_vehicles(self, num, start_point:carla.Waypoint):
        fwv = self.ego_vehicle.get_transform().get_forward_vector()
        tm = self.client.get_trafficmanager()
        bp_lib = self.world.get_blueprint_library()
        vehicles = bp_lib.filter("vehicle.*")
        return_vechicles = []
        n=0
        while n < num:
            wp: carla.Waypoint = start_point
            if wp.lane_change == carla.LaneChange.NONE:
                # No lane change, so spawn vehicles in the same lane
                location = wp.next(random.uniform(5,20))[0].transform.location
                location.z += 0.2
                transform = carla.Transform(location, self.ego_vehicle.get_transform().rotation)
                v = random.choice(vehicles)
                while v.id == "vehicle.nissan.patrol":
                    v = random.choice(vehicles)
                v.set_attribute('role_name', 'autopilot')
                v_tmp = self.world.try_spawn_actor(v, transform)
                if v_tmp is not None:
                    v_tmp.set_autopilot(True)
                    return_vechicles.append(v_tmp)
                    n+=1
            else:
                # Lane change is possible, so randomly choose to change lane or not
                if random.random() < 0.5:
                    # Change lane
                    if wp.lane_change == carla.LaneChange.Left:
                        wp = wp.get_left_lane()
                    elif wp.lane_change == carla.LaneChange.Right:
                        wp = wp.get_right_lane()
                # Spawn vehicle in the chosen lane
                location = wp.next(random.uniform(5,20))[0].transform.location
                location.z += 0.2
                transform = carla.Transform(location, self.ego_vehicle.get_transform().rotation)
                v = random.choice(vehicles)
                while v.id == "vehicle.nissan.patrol":
                    v = random.choice(vehicles)
                v.set_attribute('role_name', 'autopilot')
                v_tmp = self.world.try_spawn_actor(v, transform)
                if v_tmp is not None:
                    v_tmp.set_autopilot(True)
                    return_vechicles.append(v_tmp)
                    n+=1
        return return_vechicles

    def handle_semantic_tag(self):
        if self.tick == self.target_tick:
            # lane.id is a value that identify the lane. If we have four lanes, as in MAP 10, the lane id will be:
            # | 2 | 1 | -1 | -2 |
            # Now we have to understand if the two vehicles are in the same lane or in different lanes
            if self.wp_other_v.lane_id == self.wp_target_v.lane_id:
                #Same lane, so we make the target vehicle go faster and crash into the other vehicle
                self.tm.set_desired_speed(self.target_v, 300)
            else:
                #Different lane, WE SUPPOSE THAT THERE ARE ONLY 2 LANES FOR EACH DIRECTION, so | 2 | 1 | -1 | -2 |
                if self.wp_target_v.lane_change == carla.LaneChange.Left:
                    #Target vehicle can change lane only to the left, so the other vehicle is on the right lane
                    self.tm.force_lane_change(self.target_v, False)
                    self.tm.set_desired_speed(self.target_v, 300)
                elif self.wp_target_v.lane_change == carla.LaneChange.Right:
                    #Target vehicle can change lane only to the right, so the other vehicle is on the left lane
                    self.tm.force_lane_change(self.target_v, True)
                    self.tm.set_desired_speed(self.target_v, 300)

        if len(self.coll_queue) != 0 :
            coll_event: carla.CollisionEvent = self.coll_queue.pop()
            # The collision event is triggered when the target vehicle collides with something
            # So we stop both the vehicles and set them as dynamic anomaly
            coll_event.actor.set_autopilot(False)
            coll_event.other_actor.set_autopilot(False)
            coll_event.actor.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1, hand_brake=True))
            coll_event.other_actor.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1, hand_brake=True))
            coll_event.actor.set_actor_semantic_tag("dynamic_anomaly")
            coll_event.other_actor.set_actor_semantic_tag("dynamic_anomaly")
            print(coll_event)

    def spawn_anomaly(self):
        # Find two vehicles in front of the ego vehicle, then make them crash into each other
        vehicles = self.find_objs_in_front_ego_vehicle("vehicle.*", min_distance=2, max_distance=30, angle=0.9)
        #This filter the vehicles that are going in the same direction of the ego vehicle
        filtered_vehicles = list(
            filter(lambda v: v.get_transform().get_forward_vector().dot(
                self.ego_vehicle.get_transform().get_forward_vector()) > 0.9,
                   vehicles))
        if len(filtered_vehicles) < 2:
            print("Crash_Anomaly -> No vehicles found in front of the ego vehicle to make them crash into each other. Creating two vehicles...")
            spawned_vehicles = self.spawn_vehicles(2-len(filtered_vehicles),self.world.get_map().get_waypoint(self.ego_vehicle.get_location()))
            self.world.tick()
            vehicles = self.find_objs_in_front_ego_vehicle("vehicle.*", min_distance=2, max_distance=30, angle=0.9)
            filtered_vehicles = list(
                filter(lambda v: v.get_transform().get_forward_vector().dot(
                    self.ego_vehicle.get_transform().get_forward_vector()) > 0.9,
                       vehicles))
            filtered_vehicles.sort(key = lambda v: v.get_location().distance(self.ego_vehicle.get_location()))
        #The vehicle in position 0 will be the closest to the ego vehicle
        self.target_v = filtered_vehicles[0]
        self.tm:carla.TrafficManager = self.client.get_trafficmanager()
        #Now self.target_v will ignore alle the other vehicles
        self.tm.ignore_vehicles_percentage(self.target_v,100)
        #Also make the other vehicles ignore the target vehicle
        for v in filtered_vehicles[1:]:
            self.tm.collision_detection(v, self.target_v, False)

        # E POI FARE IN MODO CHE TARGET V TOCCHI UN ALTRO VEICOLO
        self.other_v = filtered_vehicles[1]
        self.wp_other_v:carla.Waypoint = self.world.get_map().get_waypoint(self.other_v.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        self.wp_target_v : carla.Waypoint = self.world.get_map().get_waypoint(self.target_v.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        print("Target vehicle:", self.target_v, " in lane:", self.wp_other_v.lane_id, " at location:", self.target_v.get_location())
        print("Other vehicle:", self.other_v, " in lane:", self.wp_target_v.lane_id, " at location:", self.other_v.get_location())

        self.coll_sen: carla.Sensor = attach_collision_sensor(None, self.world, self.client, self.target_v)
        self.coll_queue = deque(maxlen=1)
        self.coll_sen.listen(lambda data: self.coll_queue.append(data))

        bp_lib = self.world.get_blueprint_library()
        anomaly = bp_lib.filter(f"*{self.name}")[0]
        self.anomaly = self.world.spawn_actor(anomaly, carla.Transform(carla.Location(0,0,0),carla.Rotation(0,0,0)), attach_to=self.target_v, attachment_type=carla.AttachmentType.Rigid)
        return anomaly

    def on_destroy(self):
        super().on_destroy()