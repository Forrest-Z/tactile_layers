
This is a costmap plugin to show the position of the obstacle in the costmap when a collision is detected.

addVariable.cpp

This is the old code that update the cost of a square area. 
Not working very well. 

tactile_layer.cpp
- Updates the cost of a point
- Uses the inflation area to update the cost of a circular area.
- The attached video shows how this layer works using this code

	#Note: The inflation radius can be changed from rqt_reconfigure

	#Note: In the local costmap parameter file, you must include both the tactile layer and 	inflation area to run this.

Add the following:

    plugins: 
        - {name: tactile_layer,   type: "tactile_layer_namespace::TactileLayer", output: "screen"}  
        - {name: inflation_layer, type: "costmap_2d::InflationLayer", output: "screen"}   

Problems:
- The obstacle is not fixed on the costmap. Will follow the robot around.
- If long delay is used, it will result in a lower map update rate.

Find out more on costmaps:
- https://ros-planning.github.io/navigation2/tutorials/docs/writing_new_costmap2d_plugin.html 
- http://wiki.ros.org/costmap_2d/Tutorials/Creating%20a%20New%20Layer
