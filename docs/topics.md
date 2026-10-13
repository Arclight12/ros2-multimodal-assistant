# Topics

| Topic | Type | Publisher / purpose |
|---|---|---|
| `/workspace/image_raw` | `sensor_msgs/Image` | fixed phone stream |
| `/object_detections_raw` | `assistant_msgs/DetectedObjectArray` | detector image-space output |
| `/object_detections` | `assistant_msgs/DetectedObjectArray` | calibrated live detections |
| `/gaze/image_raw` | `sensor_msgs/Image` | laptop webcam |
| `/gaze/point` | `assistant_msgs/GazePoint` | normalized workspace gaze |
| `/final_selection` | `assistant_msgs/ObjectSelection` | temporally confirmed target |
| `/workspace_status` | `std_msgs/String` | calibration/scan status |
| `/arm/status` | `std_msgs/String` | arm availability/status |
| `/arm/command` | `std_msgs/String` | internal abstract arm command |
| `/voice/object_id` | `std_msgs/String` | voice_node -> selection_manager_node: spoken object class |

`DetectedObject` carries a stable ID, class, confidence, image center, and an
optional pose. The detector publishes image centers; the mapper fills pose only
when a valid calibration is available.
