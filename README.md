# Real-time Asynchronous Multi-Object Tracking with an Event Camera (AEMOT) ROS2 Node

> Now in real-time!

This is currently a work in progress.

This package takes the core of the AEMOT code (developed by Angus Apps, Ziwei Wang, Vladimir Perejogin, Timothy L. Molloy, and Robert Mahony) and configures it for real-time event data input and tracking using a ROS2 node.

You can see the original code on GitHub [HERE](https://github.com/angus-apps/AEMOT)

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
Since the original AEMOT code was written to 
