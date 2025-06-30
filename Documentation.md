# C++
I’ve made some modifications to the C++ source code of Carla. Here’s a list:
### MakeBlueprintDefinition() in ActorBlueprintFunctionLibrary.cpp
When a Blueprint is spawned, the class (of the blueprint) was not loaded, so I modified the code in order to load the class.
### Added a new Python API function that allows to change the semantic tag of the actors dynamically (set_actor_semantic_tag)
Here’s how it works.
1. Add the function in the Actor.cpp (In the PythonAPI folder) in a .def format
2. This calls SetActorSemanticTag() placed in Simulator.h, which has a reference to a Client instance, and uses it to call SetActorSemanticTag() placed in Client.cpp
3. The Client is used to communicate with the server through some “messages", in this case the Client sends a message containing: set_actor_semantic_tag, the actor ID and the new Tag.
4. The Server (CarlaServer.cpp) has a binding between the message set_actor_semantic_tag and a function that calls SetSemanticTag placed in CarlaActor.cpp, which call SetActorTag in Tagger.cpp 

# Python
### Main.py
This file contains all the logic for connecting to the Carla server, spawning objects, and the for-loop simulation. The simulation starts with the mustang as an ego vehicle, some other vehicles and some pedestrians. If specified, anomalies are spawned. The simulation can stop for three main causes:
1. Ego vehicle collides with an anomaly
2. Ego vehicle surpass the last anomaly
3. The simulation time is reached
The main can be launched with this command: python main.py and there are several arguments. Here’s a table describing them:

| Argument                | Description                                                                                                                                                                                           | Default Value |
|-------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------|
| -h, --help              | Show help message and exit                                                                                                                                                                            | -             |
| --seed                  | Seed for the random number generator                                                                                                                                                                  | 42            |
| --numeber_of_vehicles   | Number of vehicles to spawn for each simulation                                                                                                                                                       | 10            |
| --number_of_pedestrians | Number of pedestrians to spawn for each simulation                                                                                                                                                    | 10            |
| --filterv               | The filter used to spawn the vehicles, this doesn’t affect the ego vehicle, which is always the mustang                                                                                               | vehicle.*     |
| --filterw               | The filter used to spawn the pedestrians                                                                                                                                                              | walker.*      |
| --hybrid | If enabled, physics is only computed for a certain radius with center in the ego vehicle (more computational efficiency). A vehicle far from the ego vehicle doesn’t move smoothly, but is teleported | -             |
| --image_size_x          | Width of the camera image (for rgb, semantic, instance and depth camers)                                                                                                                              | 800           |
| --image_size_y          | Height of the camera image (for rgb, semantic, instance and depth camers)                                                                                                                             | 600           |
| --rgb                 | If enabled, the rgb camera is spawned                                                                                                                                                                 | -             |
| --semantic            | If enabled, the semantic camera is spawned                                                                                                                                                            | -             |
| --instance            | If enabled, the instance camera is spawned                                                                                                                                                            | -             |
| --depth              | If enabled, the depth camera is spawned                                                                                                                                                               | -             |
| --lidar_semantic      | If enabled, the semantic lidar is spawned                                                                                                                                                             | -             |
| --lidar | If enabled, the lidar is spawned                                                                                                                                                                      | -             |
| --radar | If enabled, the radar is spawned                                                                                                                                                                      | -             |
| --number_of_runs         | Number of runs to perform, each run is a simulation with the same parameters                                                                                                                          | 1             |
| --run_time            | Time in seconds for each run, after this time the simulation stops                                                                                                                                    | 10            |
| --anomalies | List of anomalies to spawn, separated by spaces                                                                                                                                                       | None          |   
#### Functions: 

`handle_sensor_data(args, run, sensors)`, calls the handle method for each sensor spawned in the simulation, which is present in `sensors`

`set_up_sensors(args, client,ego_vehicle,sensors,world)` , based on the sensors present in `args`, spawns the sensor and attaches it to `ego_vehicle` and inserts the spawned sensor in the `sensors` array

`generate_anomaly_object(world, client, ego_vehicle, name, distance, where, direction, anomaly_in_waypoint)`, creates the anomaly object based on the `name`

### anomalies.py
This file contains the definition of the anomalies that can be spawned in the simulation. Each anomaly is a class that inherits from `Anomaly`, which has a `spawn_anomaly()` method that spawns the anomaly in the world. The anomalies are:
- `Labrador_Anomaly`: Spawns a Labrador dog in the world, specifically on the sidewalk, that runs towards the ego vehicle.
  - `handle_semantic_tag(self)` : Sets the semantic tag of the Labrador to `Anomaly` when the dog is getting close to the street.
....

### sensors.py
This file contains the definition of the sensors that can be spawned in the simulation. Each sensor is a class that inherits from `Sensor`, which has a `handle()` method that handles the data received from the sensor. 
Sensor has the following attributes:

