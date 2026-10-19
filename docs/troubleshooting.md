# Troubleshooting & Operations Guide

This guide documents the root causes, diagnostic steps, and solutions for the real-world issues encountered when running the multimodal gaze/voice object selection and robotic arm pick pipeline.

---

## Table of Contents
1. [Quick Execution](#quick-execution)
2. [Serial Port & Arduino Communication](#serial-port--arduino-communication)
3. [PCA9685 Servo Driver & Power Pitfalls](#pca9685-servo-driver--power-pitfalls)
4. [Arm Kinematics & Reach Limits](#arm-kinematics--reach-limits)
5. [Voice Recognition & Audio Stream](#voice-recognition--audio-stream)
6. [Vision, Camera Streams & Homography](#vision-camera-streams--homography)
7. [Diagnostic Scripts & Testing](#diagnostic-scripts--testing)

---

## Quick Execution

We provide a unified runner script [`scripts/run_pipeline.sh`](../scripts/run_pipeline.sh):

```bash
# Terminal 1: Launch full pipeline (Voice + Vision + Serial Arm Controller)
./scripts/run_pipeline.sh

# Terminal 2: Monitor live object detections in real-time
./scripts/run_pipeline.sh monitor
```

### Manual Launch Equivalent
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export NVIDIA_API_KEY="<your-riva-api-key>"

ros2 launch assistant_bringup detect_pick.launch.py \
  workspace_camera_url:=http://192.168.1.2:8080/video \
  voice_stream_url:=http://192.168.1.2:8080/audio.wav \
  auto_pick:=true \
  mock_mode:=false
```

---

## Serial Port & Arduino Communication

### Issue 1: "Device or resource busy" / Port `/dev/ttyACM0` Locked
- **Symptom**: `SerialException: [Errno 16] Device or resource busy: '/dev/ttyACM0'` or Arduino upload fails with `avrdude: ser_open(): can't open device "/dev/ttyACM0"`.
- **Cause**: A previous ROS 2 node (`arm_controller_node`), diagnostic script (`test_continuous_arc_pick.py`), or background process still has `/dev/ttyACM0` open.
- **Solution**:
  ```bash
  # Check which process is holding the port
  lsof /dev/ttyACM0

  # Clean kill all assistant and ros2 launch processes
  pkill -9 -f "assistant_" || true
  pkill -9 -f "ros2 launch" || true
  sleep 1
  lsof /dev/ttyACM0  # Should be empty
  ```

### Issue 2: Flashing Firmware
- Always terminate any running ROS 2 nodes or serial monitors before flashing:
  ```bash
  pkill -9 -f "assistant_" || true
  arduino-cli compile --fqbn arduino:avr:uno firmware/continuous_arc_pick
  arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno firmware/continuous_arc_pick
  ```
- **Firmware in Use**: [`firmware/continuous_arc_pick/continuous_arc_pick.ino`](../firmware/continuous_arc_pick/continuous_arc_pick.ino). On boot, it performs an initial calibration and outputs `ARM_READY`.

---

## PCA9685 Servo Driver & Power Pitfalls

### Issue 1: I2C Bus Hangs / Arduino Freezes During Pick Cycle
- **Symptom**: The arm begins moving, then stops mid-arc. The Arduino stops responding to serial commands, and `CYCLE_COMPLETE` is never sent.
- **Cause**: The standard Arduino `Wire` library blocks indefinitely in `Wire.endTransmission()` if the PCA9685 loses communication or suffers a voltage sag.
- **Solution**:
  1. The firmware includes an explicit I2C bus timeout:
     ```cpp
     Wire.begin();
     Wire.setWireTimeout(25000, true); // 25ms timeout with reset on hang
     ```
  2. **Wiring Check**:
     - **VCC (PCA9685)**: Connect to **5V pin on Arduino** (powers the PCA9685 chip logic).
     - **V+ (PCA9685 screw terminal)**: Connect to an **external regulated 5V (3A–5A) power supply** (powers the servo motors).
     - **GND**: Must be common between Arduino Uno, PCA9685, and external 5V power supply.
     - **NEVER** power servos directly from the Arduino Uno 5V regulator.

### Issue 2: Servo Jitter and Brownout Resets
- **Symptom**: Arduino reboots (serial prints `ARM_READY` unexpectedly) when servos start moving.
- **Cause**: High peak inrush current from multiple SG90/MG90S servos causing voltage drops on the power rails.
- **Solution**: Ensure a large decoupling capacitor (at least $1000\,\mu\text{F}$, 10V–16V) is installed across the V+ and GND terminals of the PCA9685 board.

---

## Arm Kinematics & Reach Limits

### Issue 1: Arm Refuses to Pick Far Objects ("Out of Reach")
- **Symptom**: `[pick_trigger_node] Target (x, y) is out of continuous reach: radius ... > max ...`
- **Cause**: Physical kinematics limit. The MeArm linkage can safely reach a radius of $28.5\,\text{cm}$ ($0.285\,\text{m}$). Previously, `R_MAX_SAFE_M` was conservatively set to $0.26\,\text{m}$, rejecting objects placed near the outer edges.
- **Solution**:
  - `R_MAX_SAFE_M = 0.285` in [`src/assistant_motion/assistant_motion/kinematics.py`](../src/assistant_motion/assistant_motion/kinematics.py).
  - Place objects within the valid operating zone:
    - Radial distance: $12\,\text{cm} \le R \le 28.5\,\text{cm}$.
    - Base angle: $-45^\circ \le \theta \le +45^\circ$ relative to arm centerline.

### Issue 2: Discrete Slot Fallback Mechanism
- If a target coordinate is slightly outside the continuous inverse kinematics solver range, [`pick_trigger_node.py`](../src/assistant_motion/assistant_motion/pick_trigger_node.py) automatically attempts a fallback:
  1. Primary: Continuous Cartesian Arc Pick (`P,<base>,<link1>,<link2>,<grip>`).
  2. Secondary: Discrete Slot Pick (`S,<slot_id>`) mapped from the closest pre-calibrated slot.

---

## Voice Recognition & Audio Stream

### Issue 1: ASR Flooded with Silent Recordings / False Triggers
- **Symptom**: Voice node repeatedly triggers ASR every 5–6 seconds with background silence, leading to high latency and Riva API rate limiting.
- **Cause**: In [`src/assistant_interaction/assistant_interaction/audio_stream.py`](../src/assistant_interaction/assistant_interaction/audio_stream.py), `Segmenter._noise` was not continuously adapting to ambient acoustic shifts.
- **Solution**:
  - Implemented exponential moving average update in `Segmenter.feed()`:
    ```python
    self._noise = 0.95 * self._noise + 0.05 * rms
    ```
  - Speech threshold is determined dynamically relative to ambient noise (`rms > self._noise * speech_ratio + min_floor`).
  - Do **not** hardcode a fixed high RMS cutoff (such as 2500) as it prematurely cuts off trailing consonants in phrases like "pick object a".

### Issue 2: Riva ASR Transcription Phonetic Confusions
- **Symptom**: User says "pick object b", but Riva ASR returns `"pick objective"` or `"pick object to"`, causing `selection_manager` to drop the command.
- **Solution**:
  - Added phonetic aliases in [`src/assistant_interaction/assistant_interaction/voice_command.py`](../src/assistant_interaction/assistant_interaction/voice_command.py):
    ```python
    PHONETIC_ALIASES = {
        "objective": "block_b",
        "object to": "block_b",
        "object be": "block_b",
        "object see": "block_c",
    }
    ```

---

## Vision, Camera Streams & Homography

### Issue 1: Camera Stream Latency or Dropouts
- **Symptom**: Detections lag several seconds behind physical movements.
- **Solution**:
  1. In the **IP Webcam** app on the phone:
     - Resolution: Set to **1280x720** or **640x480** (do not use 1080p/4K).
     - FPS Limit: 30 FPS.
     - Video Quality: 50–70%.
  2. Verify network ping:
     ```bash
     ping 192.168.1.2
     ```
     Ensure round-trip time is $< 20\,\text{ms}$.

### Issue 2: Lighting & Marker Detection
- **Symptom**: Homography projection is distorted or objects are detected at incorrect $(x, y)$ world coordinates.
- **Solution**:
  - Homography transformation matrix is configured in [`src/assistant_bringup/config/perception.yaml`](../src/assistant_bringup/config/perception.yaml).
  - Avoid direct sunlight glare or harsh shadows across the calibration board dots / ArUco markers.
  - To inspect live bounding box and workspace mappings:
    ```bash
    ros2 topic echo /object_detections
    ```

---

## Diagnostic Scripts & Testing

Before running the full ROS 2 graph, verify hardware subsystems independently:

| Subsystem | Diagnostic Script | Expected Output |
|---|---|---|
| **Full Unit Tests** | `pytest tests/` | `43 passed` |
| **Arm Servos** | `python3 scripts/test_arm_servos.py` | Individual joint pulse testing |
| **Continuous Pick** | `python3 scripts/test_continuous_arc_pick.py` | Tests full pick-and-place cycle on `/dev/ttyACM0` |
| **Voice Streaming** | `python3 scripts/test_voice_pick.py` | Validates phone WAV stream and Riva transcription |
| **Camera Feed** | `python3 scripts/check_cameras.py --workspace-url http://192.168.1.2:8080/video` | Confirms frame capture and FPS |

---

## Emergency Arm Reset

If the arm binds, stalls, or makes a buzzing sound:
1. **Immediately turn off servo power** (external power supply switch).
2. Stop ROS 2:
   ```bash
   pkill -9 -f "assistant_" || true
   pkill -9 -f "ros2" || true
   ```
3. Gently move the acrylic linkages by hand back to the Home pose (vertical / upright).
4. Turn on servo power and re-launch.
