
Tactile layer
===================================================================================================================
This is a costmap plugin used to reflect tactile sensor data on the local costmap.
Tactile sensor msg -> Wrenchstamped

How it works?
=====================================================================================================================
Using force x and y from the tactile sensor with trigonometry to calculate the position of collision.

This calculated position is one cell only, where inflation layer is required to inflate this cell to properly represent
a person. 

A delay of 1s is placed to show the collision on costmap

Problems
=====================================================================================================================
1. Collision position is calculated wrt robot so, when robot moves, collision position also follows. this is due to the
updating feature of the local costmap.

2. Inflation layer is needed for this to work properly. An average radius of a person is approx. 0.4m limits the robot's 
ability to move around in tight places and corridors. 

3. The delay placed to show the collision slows the update rate of the costmap which is not recommended since the robot will 
not get new information especially when its trying to avoid the collision.


addVariable.cpp
=====================================================================================================================

This is the old code that update the cost of a square area. 
Not working very well. 

tactile_layer.cpp
=====================================================================================================================
- Updates the cost of a point
- Uses the inflation area to update the cost of a circular area.
- The attached video shows how this layer works using this code

	#Note: The inflation radius can be changed from rqt_reconfigure

	#Note: In the local costmap parameter file, you must include both the tactile layer and 	inflation area to run this.

Add the following:

    plugins: 
        - {name: tactile_layer,   type: "tactile_layer_namespace::TactileLayer", output: "screen"}  
        - {name: inflation_layer, type: "costmap_2d::InflationLayer", output: "screen"}   

Find out more on costmaps:
- https://ros-planning.github.io/navigation2/tutorials/docs/writing_new_costmap2d_plugin.html 
- http://wiki.ros.org/costmap_2d/Tutorials/Creating%20a%20New%20Layer
