# Real-time 'Asynchronous Multi-Object Tracking with an Event Camera' (AEMOT) ROS2 Node

> Now in real-time!

This is currently a work in progress.

This package takes the core of a reduced version of the AEMOT code (developed by Angus Apps, Ziwei Wang, Vladimir Perejogin, Timothy L. Molloy, and Robert Mahony) and configures it for real-time event data input and tracking using a ROS2 node.

You can see the original full code on GitHub [HERE](https://github.com/angus-apps/AEMOT). The exact, reduced, code that this package here was based on was provided directly by one of the original authors.

## How to install and build
This package is built for an environment running ***ROS2 Jazzy*** (requires ***Ubuntu 24.04***). However, it should be able to run with little to no changes on later versions of ROS2 (such as *ROS2 Lyrical* on *Ubuntu 26.04*). This is yet to be tested, though. 

<ins>**Step 1: Create directory**</ins>  
If you are using this package as a part of a larger meta-package, then the structure will need to be as so:
```txt
-- working_directory
      |
      -- src
          |
          -- <other_packages>
          -- ...
          -- aemot_ros2
              |
              -- src
              -- configs
              -- CMakeLists.txt
              -- package.xml
```
If this is the case, then navigate to your overarching `working_directory/src` folder, create a new `aemot_ros2` folder, and clone this repo inside that:
```bash
# navigate to main src folder:
cd working_directory/src
# create new package folder
mkdir aemot_ros2
# clone this repo here (make sure you include the '.' at the end of the final line here!)
cd aemot_ros2
git clone https://github.com/TobysWorkshop/AEMOT-ros2.git . 
```

If you are just using this package standalone, the best practice would be to create the same structure as above:
```bash
mkdir -p working_directory/src/aemot_ros2
cd working_directory/src/aemot_ros2
git clone https://github.com/TobysWorkshop/AEMOT-ros2.git . 
```

<ins>**Step 2: Install dependencies**</ins>  
WIP

<ins>**Step 3: Build!**</ins>  
```bash
cd working_directory
colcon build
# or use colcon build --packages-select aemot_ros2 to install just this one package
```

## Key amendments to the original code
The original AEMOT code has 4 key central components:
* main : the event data inject and association/distribution. Calls the SAEdetector to determine whether an event is part of an existing object, a new object, or background noise, and calls the TrackManager to update tracks accordingly.
* TrackManager : the track lifetime manager. Creates, updates, and destroys kalman instances to hold each track's state.
* kalman : the core of the tracking logic - holds a kalman filter state for each object.
* SAEdetector : determines whether an event is part of an existing object, a new object, or background noise.

Since the original AEMOT code was written to pull events from a file, ***`main.cpp`*** is the main piece to have been changed in this real-time package. Now, ***`main_node.cpp`*** has instead been set up as a ROS2 node to pull events from a dedicated event_camera/events topic to ingest into the system. A lot of the unnecessary non-real-time functionality of main.cpp has also been stripped. 

To save resources on the real-time model, TrackManager has now been changed to create and manage a set pool of kalman filters at the start of the run. The old code created and destroyed kalman filter classes every time a new track started and ended. Now, a pool of, say, 50 kalman filter classes are initialised at the start and set to inactive. Once a track starts, a kalman filter is pulled off the lineup and set to active. Once a track ends, its corresponding kalman filter is reset and deactivated, ready for reuse. This saves the drastic resource drains of constantly creating and decreating classes at run-time.

This also means that slight changes to the kalman class have been made to include a reset function and active/inactive tags.

...WIP...
