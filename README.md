
This is a costmap plugin to show the position of the obstacle in the costmap when a collision is detected.

- addVariable.cpp

This is the old code that update the cost of a square area. 
Not working very well. 

- tactile_layer.cpp

  -Updates the cost of a point
  -Uses the inflation area to update the cost of a circular area.
  -The attached video shows how this layer works using this code

#Note: The inflation radius can be changed from rqt_reconfigure
#Note: In the local costmap parameter file, you must include both the tactile layer and inflation area to run this.

Add the following:

    plugins: 
        - {name: tactile_layer,   type: "tactile_layer_namespace::TactileLayer", output: "screen"}  
        - {name: inflation_layer, type: "costmap_2d::InflationLayer", output: "screen"}   
