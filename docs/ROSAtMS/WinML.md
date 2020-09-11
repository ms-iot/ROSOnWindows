# Windows ML ROS Node
The AI platform in Windows 10 enables developers to use pre-trained machine learning models in their Apps on Windows devices. This offers developers a number of benefits:

* Low latency, real-time results. Windows can perform AI evaluation tasks using the local processing capabilities of the PC, enabling real-time analysis of large local data such as images and video. Results can be delivered quickly and efficiently for use in performance intensive workloads like game engines, or background tasks such as indexing for search.
* Reduced operational costs. Together with Microsoft’s Cloud AI platform, developers can build affordable, end-to-end AI solutions that combine training models in Azure with deployment to Windows devices for evaluation. Significant savings can be realized by reducing or eliminating costs associated with bandwidth due to ingestion of large data sets, such as camera footage or sensor telemetry. Complex workloads can be processed in real-time on the edge, with minimal sample data sent to the cloud for improved training on observations.
* Flexibility. Developers can choose to perform AI tasks on device or in the cloud based on what their customers & scenarios need. AI processing can happen on the device if it becomes disconnected, or in scenarios where data cannot be sent to the cloud due to cost, size, policy or customer preference. 

## Consuming WinML
Requirements:

* Install Visual Studio 2019 with UWP development
* ROS melodic for Windows


The WinML ROS Node is distrubted as source. To consume it in your robot, clone the winml_tracker sources into your workspace.

For example:
```bat
mkdir c:\workspace\winml_demo\src
cd c:\workspace\winml_demo\src
catkin_init_workspace
git clone https://github.com/ms-iot/winml_tracker
```

Create a Launch file, which references the model.onnx file:
```xml
<launch>
  <arg name="onnx_model_path_arg" default="$(find winml_tracker)/testdata/model.onnx"/>
  <node pkg="winml_tracker" type="winml_tracker_node" name="winml_tracker" output="screen">
    <param name="onnx_model_path" value="$(arg onnx_model_path_arg)"/>
    <param name="confidence" value="0.5"/>
    <param name="tensor_width" value="416"/>
    <param name="tensor_height" value="416"/>
    <param name="tracker_type" value="yolo"/>
    <param name="image_processing" value="resize"/>
    <param name="debug" value="true"/>
    <param name="image_topic" value="/cv_camera/image_raw" />
  </node>
  
  <!-- NOTE: The image properties need to be valid for the camera, or the node will auto select the closest values -->
  <node pkg="cv_camera" type="cv_camera_node" name="cv_camera" output="screen">
    <param name="rate" type="double" value="5.0"/>
    <param name="image_width" type="double" value="640"/>
    <param name="image_height" type="double" value="480"/>
  </node>

  <node pkg="tf" type="static_transform_publisher" name="winml_link"
    args="0 -0.02  0 0 0 0 map base_link 100" />  

</launch>
```

> While 'Pose' processing is enabled, the service required to generate the model has not been published as of October 2020
 

## Property Descriptions

| Property | Description |
|----------| ------------|
| onnx_model_path | Path to the model.onnx file | 
| confidence | Minimum confidence before publishing an event. 0 to 1 |
| tensor_width| The Width of the input to the model. |
| tensor_height| The Height of the input to the model. |
| tracker_type| Currently enabled - `yolo` or `pose`. |
| image_processing| `resize`, `scale` or `crop` |
| debug| `true` or `false` determines if a debug image is published |
| image_topic| The image topic to subscribe to |
| label | used to filter the found object to a specific label |
| mesh_rotation| The orientation of the mesh when debug rendering pose |
| mesh_scale| The scale of the mesh when debug rendering pose |
| mesh_resource| The mesh used for debug rendering pose |
| model_bounds| 9 coordinates used to perform the point in perspective caluclation for pose |
| calibration | Path to the OpenCV calibration file for point in persective |

## Subscriptions
WinML subscribes to the topic listed in the `image_topic` property, or `/cv_camera/image_raw`


## Publishing
The WinML Publishes the following topics:

### /tracked_objects/image
Outputs an image with highlighing if the debug property is set

### /tracked_objects/
An array of `visualization_msgs::Marker` for found objects

### /detected_object
A single instance of the DetectedObjectPose message, which is output when tracker_type is set to pose.

