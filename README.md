# ROS WRAPPER FOR
# RAISIM, A PHYSICS ENGINE FOR ROBOTICS AND AI RESEARCH (v1.10.1)

## Build

```
cd <path_to_ws>/catkin_ws
mkdir src
cd src
git clone <this_repo_url>
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

## Launch
```
roslaunch raisim unity.launch
roslaunch raisim opengl.launch
```

It's forked from [Git](https://github.com/raisimTech/raisimLib)

Documentation available on the [RaiSim Tech website](http://raisim.com).

## License

You should get a valid license and an activation key from the [RaiSim Tech website](http://raisim.com) to use RaiSim.
Post issues to this github repo for questions.
Send an email to info.raisim@gmail.com for any special inquiry.

## Supported OS

Linux
