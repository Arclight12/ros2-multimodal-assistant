# Voice input

Say "pick up the cup" and the arm targets the cup, beating gaze for
`voice_priority_seconds` (default 5). The Pi needs no microphone: the phone is
the mic and the Pi does all the processing.

## How it works

1. IP Webcam on the phone serves live audio at `http://<phone-ip>:8080/audio.wav`.
2. `voice_node` (on the Pi) cuts the stream into utterances by loudness
   (`Segmenter`, `audio_stream.py`). Silence is never sent.
3. Each utterance is transcribed by NVIDIA's hosted speech NIM (Nemotron streaming ASR) over a
   persistent gRPC channel (`nvidia-riva-client`).
4. `match_class` (`voice_command.py`) maps the words onto a detector class
   ("mug" -> `cup`). A match is published on `/voice/object_id`;
   `selection_manager_node` selects the most confident visible detection of it.

Edit the words in `DEFAULT_ALIASES` (`voice_command.py`); the class names must
equal the detector model's class names.

## Setup on the Pi

```bash
python3 -m pip install -r requirements.txt     # adds nvidia-riva-client
export NVIDIA_API_KEY=nvapi-...                 # never stored in the repo
colcon build --symlink-install --packages-select assistant_interaction assistant_bringup
ros2 launch assistant_bringup system.launch.py mock_mode:=false   voice_stream_url:=http://<phone-ip>:8080/audio.wav
```

Phone: same Wi-Fi as the Pi, IP Webcam running with microphone permission, screen
on, battery optimisation off for the app.

## Parameters (`config/voice.yaml`)

| Parameter | Purpose |
|---|---|
| `voice_stream_url` | phone audio URL |
| `nvidia_function_id` | which hosted ASR model; copy from its build.nvidia.com API page |
| `voice_min_rms` | loudness that counts as speech; raise it in a noisy room |
| `voice_end_silence_ms`, `voice_max_utterance_s` | end-of-speech quiet time (600 ms), longest utterance (6 s) |

Mock mode starts nothing. Test the selection path by hand:

```bash
ros2 topic pub --once /voice/object_id std_msgs/String "data: cup"
```

## Troubleshooting

- `voice_stream_url or NVIDIA_API_KEY is missing`: export the key in the launch shell.
- `nvidia-riva-client is not installed`: run the pip command above.
- `NVIDIA speech request failed`: the log line carries the gRPC status (bad key,
  wrong function id, no network).
- `Heard '...': no object matched`: add the word to `DEFAULT_ALIASES`.
- Nothing is heard in a loud room: lower `voice_min_rms`.

Live-tested from a laptop with synthesized speech (default function id works,
about 0.3 s per utterance after the first call); phone audio quality and the Pi
install are not yet verified.
