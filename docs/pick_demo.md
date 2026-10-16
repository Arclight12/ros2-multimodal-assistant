# Pick demo: gaze or voice chooses, the Arduino fetches

Four objects sit in four fixed positions. Gaze (looking at an object) or voice
("pick up the pen") chooses one; `selection_manager_node` publishes it on
`/final_selection`, and `pick_trigger_node` turns its class into a label that
goes to the Arduino. The Arduino swings the base to that slot, picks the object
up, carries it home, holds 3 s and drops it at `DROP_BASE`. Voice outranks gaze
for `voice_priority_seconds`. The detector only provides the objects the gaze
and voice choose from.

| Label | Object | Serial command |
|---:|---|---|
| 0 | toy | `D,0` |
| 1 | eraser | `D,1` |
| 2 | pen | `D,2` |
| 3 | phone | `D,3` |

The order is `pick_classes` in `src/assistant_bringup/config/pick.yaml`. The
class names must equal the detector model's class names
(`models/object_detection/object_detector.pt`). The shipped model knows `cup`,
`water_bottle`, `book`, `pen`, `phone` and `ball`, so `toy` and `eraser` are
never detected until the model is retrained with those classes.

## Arduino

Flash `firmware/pick_slots/pick_slots.ino` (needs the Adafruit PWM Servo Driver
library). Set `PICKUP_BASE[4]` (measure each slot with `B,<pulse>` in the serial
monitor), `PICKUP_LINK1/2` and `DROP_BASE`. Replies: free-text progress lines
(`PICKUP_START`, `PICKUP_COMPLETE`, `HOLDING_AT_HOME`, ...), then
`CYCLE_COMPLETE` or `CYCLE_ABORTED`; `ERR,<reason>` for a refused command. `x`
stops a move. `H`, `O`, `C` and `B,<n>` send no completion reply, so only `pick`
and `stop` are sent from ROS with this firmware. Hold the arm at home before the
Arduino is powered or reset.

## Run (on the Pi)

After `colcon build --symlink-install` (new entry point), with the arm on
`serial_port` in `robot.yaml`:

```bash
export NVIDIA_API_KEY=...   # voice
ros2 launch assistant_bringup detect_pick.launch.py mock_mode:=false \
  workspace_camera_url:=http://<phone-ip>:8080/video \
  voice_stream_url:=http://<phone-ip>:8080/audio.wav \
  gaze_camera_device:=http://<laptop-ip>:8081/video gaze_camera_backend:=ffmpeg \
  auto_pick:=true
```

With `auto_pick:=false` (the default) the label is only logged and published on
`/object_label`; the arm does not move. After a pick finishes, another selection
is accepted only after `rearm_gap_s`. A selected class that is not in
`pick_classes` is ignored with a warning. Say the object's name for voice:
"toy", "eraser", "pen" or "phone" (add words in
`assistant_interaction/voice_command.py`).
