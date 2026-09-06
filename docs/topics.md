# Topics

This document describes every ROS 2 topic used by the multimodal assistant,
including message types, publishers, subscribers, and semantics.

## Topic Summary

| Topic                | Type                                    | Publish (from)        | Subscribe (to)         |
|----------------------|-----------------------------------------|-----------------------|------------------------|
| `/voice/object_id`   | `assistant_msgs/ObjectSelection`        | `voice_node`          | `selection_manager`    |
| `/gaze/object_id`    | `assistant_msgs/ObjectSelection`        | `gaze_node`           | `selection_manager`    |
| `/final_selection`   | `assistant_msgs/ObjectSelection`        | `selection_manager`   | `motion_planner`, `arm_controller` |
| `/object_detections` | `std_msgs/String` (JSON)                | `workspace_mapper`    | observers              |
| `/workspace_status`  | `std_msgs/String`                       | `workspace_mapper`    | UI / RQt              |

## `/voice/object_id`

Published by the voice node whenever a spoken command is recognized. Carries
the recognized object and the `source` field set to `"voice"`.

## `/gaze/object_id`

Published by the gaze node when the user's gaze is inferred to land on an
object. Carries the object and the `source` field set to `"gaze"`.

## `/final_selection`

Published by the `selection_manager_node` after arbitration. This is the
single **authoritative** selection consumed by the motion layer. The `source`
field records which modality produced the winning selection.

## `/object_detections`

Published by the `workspace_mapper_node` with raw detection results serialized
as JSON. This is a diagnostic/observability topic and is not required to run
the nominal pipeline.

## `/workspace_status`

Published by the `workspace_mapper_node` with text status values such as
`SCANNING`, `SCAN_COMPLETE`, and `SCAN_ERROR`. Useful for driving simple UIs
and for debugging.

## `ObjectSelection` Message

```
string object_id
string source     # voice | gaze | system
```

The `source` field constrains the allowed origins. The interaction layer is
responsible for ensuring only valid sources are emitted.
