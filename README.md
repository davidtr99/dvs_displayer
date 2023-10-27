# Dynamic Vision Sensor Displayer
This package is a ROS node that displays the events from the DVS camera.

The main dependencies are:
- ROS Noetic
- OpenCV
- dvs_msgs

## Run
To run the node, use the following command:
```
roslaunch dvs_displayer dvs_displayer.launch
```

## Parameters
Some of the ROS parameters can be set through launch file arguments. The following parameters are available:
- `events_topic`: The topic to subscribe to. Default: `/dvs/events`
- `image_topic`: The topic to publish the image to. Default: `/dvs_displayer/image_raw`
- `rqt_reconfigure`: Whether to start the rqt_reconfigure node. Default: `false`
- `rqt_image_view`: whether to start the rqt_image_view node. Default: `false`

## Rqt Reconfigure
The rqt_reconfigure node allows to change the parameters of the node during runtime. The following parameters can be changed: 
- `display_method`: The method to display the events. Possible values: `histogram`, `ternary`, `time_surface`. Default: `ternary`
- `color_map`: The color map to use. Possible values: `grayscale`, `seismic`, `viridis`. Default: `seismic`
- `frequency`: The frequency of the generated visualization image, a negative value with produce an image per event package. Default: `-1`
