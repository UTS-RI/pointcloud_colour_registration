# pointcloud_colour_registration
Utility package to use pinhole camera model to register colour to a single pointcloud from RGB cameras.
Assumes that no cameras have overlapping FOV.  

## Usage
```
roslaunch pointcloud_colour_registration colour_registration.launch
```
Note: Tested with a merged pointcloud from two Ouster OS0-128, and 3 Intel Realsense D435 RGB cameras with no overlapping FOV.
### Arguments
* ~published_topicname - Topic name of the colour pointcloud
* ~pointcloud_topicname - Topic name of the input pointcloud to colour
* ~base_frame - Frame of the input pointcloud
* ~image_transport_options - Transport options for colour camera. Options are 'raw' and 'compressed' - from image_transport package

Note: For more than one RGB camera, the following arguments are delimited by commas. E.g. /camera_pi1/color/image_raw,/camera_pi2/color/image_raw,/camera_pi3/color/image_raw
* ~camera_frames - The link frame of the colour cameras. ROS camera drivers usually default to /camera_link
* ~camera_subtopics - The image_transport topic tree for the colour image. ROS camera drivers usually default to /camera/image_raw
* ~camerainfo_topics - The camera_info topics for the camera. ROS camera drivers usually default to /camera/camera_info
* ~frame_suffix - The suffix for the optical frame where the RGB images come from. ROS camera drivers usually default to _color_optical_frame  
Frame suffix note: If only a single value is given (no comma delimiter), the same suffix is assumed for all cameras.


## Dependencies
* Eigen3 (debian release on Ubuntu 20.04)
* OpenCV (ros noetic opencv installation - opencv4)
* PCL (debian release for ros noetic - pcl 1.10)
* ROS packages
    * cv_bridge
    * image_transport
    * pcl_conversions
    * pcl_ros
    * roscpp
    * sensor_msgs
    * tf
    * tf_conversions

## TODO
* Make base_frame argument optional - read from header of input pointcloud if none given
* Add intensity values into colour pointcloud - accommodate for algorithms which rely on intensity
* Address multiple RGB cameras with overlapping FOV
