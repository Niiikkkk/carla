# How to add a vechicle to the blueprint library
1. Create the skeleton/blueprint in the Unreal Engine editor
2. Go to /Unreal/CarlaUnreal/Content/Carla/Config and open VehicleParameters.json
3. Add the new vehicle following the format of the other vehicles
4. This will automatically add the vehicle to the blueprint library
5. It's name will be "vechile.<Make>.<Model>", where <Make> and <Model> are the values you set in VehicleParameters.json

# How to add a new prop
1. Create the prop in the Unreal Engine editor (skeleton/blueprint)
2. Go to /Unreal/CarlaUnreal/Content/Carla/Config and open BlueprintParameters.json
3. Fill the fields
4. This will automatically add the prop to the blueprint library
5. It's name will be "blueprint.<Name>", where <Name> is the value you set in BlueprintParameters.json


For the semantic lidar you have to add the anomaly classes to GetTagFromString in Tagger.cpp.
For semantic segmentation, follow the online guide.
AFTER MODIFING THE CITYSCAPES PALETTE, you have to recompile the CarlaUnreal project. (not just launch, but recompile everything!!,
Use the carla setup script to do this)


MODFICHE FATTE SU 
Tagger.cpp
Tagger.h
CarlaActor.cpp
CarlaActor.h
CarlaServer.cpp
Client.cpp
Client.h
Actor.cpp (PythonAPI)
Simulator.h

Adding new function to the Carla API:
1. Add the python definition (.def) in Actor.cpp (Actor.cpp is in the PythonAPI folder) or other .cpp files in PythonAPI
2. This function should call a function in Simulator.h, which has a reference to the Client class (The client is the one that communicates with the server)
3. Define in Client a function that communicates with the server (CarlaServer.cpp)
4. In CarlaServer.cpp, bind the name of before with the function you want to call
5. Call that function...

# How to use the main.py script
### Parameters
- -h, --help            show this help message and exit
- 