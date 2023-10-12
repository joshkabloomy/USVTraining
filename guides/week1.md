
# Week 1 Guide

## 1. Set up the USVTraining repo

I recommend [forking this repository](https://docs.github.com/en/get-started/quickstart/fork-a-repo) to your personal GitHub. 

When you fork the repository, you create a copy of the repo in your personal GitHub. From this copy, you can make changes to any branch including the main branch as you'd like. As you continue expanding the functionality of the robot, it will be useful to create new branches and merge back to main, but how you use git is completely up to you.

Once you've forked the repo, clone it to an empty workspace. (e.g. you may have a file structure of `dev_ws/src/USVTraining` after cloning). It is recommended to do any builds from the root of your workspace (in this case `dev_ws`).

To build the packages, from the root of your workspace you can run `colcon build` (don't forget to source ROS!). If you have been able to build [Virtuoso](https://github.com/gt-marine-robotics-group/Virtuoso) you will have all required dependencies already installed.

Remember that as you make changes, you will need to run `colcon build` and `source install/setup.bash` each time from your workspace.

## 2. Test Given Functionality

We have provided you with localization (using robot_localization) and some basic perception (detecting obstacle bounding boxes). This week, your goal is to upgrade the existing path planning algorithm being used (literally just a straight line).

After building and sourcing, start the VRX simulation (using the default world). Make sure to pass in a path to the provided urdf file (found in `/usv/usv.urdf`) in the `urdf` launch argument of the simulation. For example,

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta urdf:=<path to given urdf>
```

Then, launch the `test_navigation.launch.py` launch file to spin up the localization, basic perception, and the navigation node. Command shown below:

```
ros2 launch training_navigation test_navigation.launch.py
```

Note that there is a launch argument that you can pass in, called `usv`. It is defaulted to `simulation`, so you won't need to worry about specifying it until you run your code on the physical robot. The launch argument is used to determine which parameters to pass into the nodes (those relevant for the physical or simulated robot). This is also what we do with Virtuoso.

If everything works, you should see RVIZ appear and look similar to the following:

![The RVIZ screen](/guides/images/usvtrainingrviz.png)

The yellow boxes are visualizations of the bounding boxes found by the current perception provided.

You'll notice that in the toolbar on the top of RVIZ, there is a tool called `2D Goal Pose`. To send a goal pose to the path planning node, click on the `2D Goal Pose` button and then click on the screen. You should see a straight path from the robot's current pose to the goal pose appear. This is the current path planning. You will notice it ignores any obstacles.

## 3. Improve the Path Planning

Your goal is to implement some path planning algorithm which will avoid the obstacles detected by the perception nodes. All code changes necessary will be done in `training_navigation/training_navigation/planner.py` and `training_navigation/training_navigation/test_navigation_node.py`. There are few comments in place to help guide you.

Essentially, you will want to create a subscriber to the bounding boxes topic and then pass those bounding boxes (which are obstacles) into your path planning algorithm in `planner.py`. You are free to implement whatever algorithm you choose.

When you want to test your algorithm, change the `planner` parameter found in `training_navigation/config/simulation/navigation.yaml` from STRAIGHT_LINE to CUSTOM. This will tell the node to use your custom planner instead of the straight line planner. The node will publish your path to the same topic as before, so you should see your path appear on RVIZ when you use the `2D Goal Pose` tool.