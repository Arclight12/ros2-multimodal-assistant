# Architecture

```text
phone stream -> workspace_camera -> object_detector -> raw detections
                                                   -> workspace_mapper
                                                        -> detections
laptop webcam -> gaze_camera -> gaze_estimation -> gaze point
                                                     \                 /
                                                      selection_manager
                                                             |
                                                        final_selection
                                                             |
                                                   motion_planner action
                                                             |
                                                   arm_controller -> arm
```

The mapper is the only component that converts image coordinates into the
workspace/base frame. The selection manager is the only component that decides
human intent. The motion package never subscribes to images and only executes
an explicit `ExecuteGrasp` action.

Normal operation is continuous; the scan service is retained for calibration
and testing and writes a runtime snapshot outside the repository.
