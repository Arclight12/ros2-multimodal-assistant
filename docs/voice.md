# Voice input

Voice lets the user name the object to grasp ("pick up the cup") instead of
looking at it. The phone is the microphone.

## How it works

1. The phone's IP Webcam app serves live audio as WAV at
   `http://<phone-ip>:8080/audio.wav`.
2. `voice_node` reads that stream and feeds it to the offline Vosk recogniser
   (`vosk-model-small-en-us-0.15`). Recognition is restricted to a small grammar
   built from the object aliases, so unrelated speech is ignored.
3. When a phrase matches an object class (`cup`, `water_bottle`, `book`, `pen`,
   `phone`, `ball`), `voice_node` publishes that class on `/voice/object_id`
   (`std_msgs/String`).
4. `selection_manager_node` consumes `/voice/object_id` and gives it priority over
   gaze:
   - If a detection of that class is currently visible, the best-confidence
     detection of the class is selected immediately with source `voice`.
   - Gaze selections that conflict with it are suppressed for
     `voice_priority_seconds` (default 5).
   - A voice command older than that is ignored. If the named object is not
     visible, the command is ignored with a warning.

Voice wins over gaze. Gaze is still used when no recent voice command exists.

## Phone setup (IP Webcam)

- Put the phone and laptop on the same Wi-Fi network.
- Open IP Webcam and tap **Start server**. Note the IP shown in the app.
- Grant the app microphone permission. Audio is not served without it.
- Keep the screen on and disable battery optimisation for the app, otherwise
  Android pauses the audio stream.
- The audio URL is `http://<phone-ip>:8080/audio.wav`. Use the direct URL, not the
  app's browser control page.

## Model download

The recogniser needs the unpacked model directory `models/voice/vosk-model-small-en-us-0.15`:

```bash
curl -LO https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip -d models/voice/
```

`models/` is git-ignored. Install the runtime dependency with
`python3 -m pip install -r requirements.txt` (the `vosk` package).

## Test without ROS

```bash
python3 scripts/check_voice.py --url http://<phone-ip>:8080/audio.wav --seconds 15
```

The script opens the stream, prints its format, and prints each heard phrase with
the matched class (or `no object matched`). Exit code is 0 if at least one phrase
was heard, 1 otherwise. Use `--model` to point at a different model directory.

## Test the ROS path

Publish a voice command by hand, without the microphone:

```bash
ros2 topic pub --once /voice/object_id std_msgs/String "data: cup"
```

Then check `/final_selection` and the selection manager's log.

## Launch

```bash
ros2 launch assistant_bringup system.launch.py mock_mode:=false \
  voice_stream_url:=http://<ip>:8080/audio.wav
```

Parameters:

| Node | Parameter | Purpose |
|---|---|---|
| `voice_node` | `voice_stream_url` | phone audio URL (`/audio.wav`) |
| `voice_node` | `vosk_model_path` | unpacked Vosk model directory |
| `voice_node` | `voice_open_timeout_s` | timeout for opening the audio stream (default 5.0) |
| `voice_node` | `voice_chunk_ms` | audio chunk size fed to Vosk (default 200) |
| `voice_node` | `voice_reconnect_base_delay_s` | first reconnect delay after the stream drops (default 1.0) |
| `voice_node` | `voice_reconnect_max_delay_s` | upper bound on the reconnect backoff (default 30.0) |
| `selection_manager_node` | `voice_priority_seconds` | how long a voice command overrides gaze (default 5) |

Mock mode (`mock_mode`, the default) does not start the recogniser or open the
stream, so `voice_node` publishes nothing in mock mode. Use the `ros2 topic pub`
command above to exercise the selection path.

## Words and aliases

Recognised phrases are the object names and aliases in `DEFAULT_ALIASES`, for
example "cup" or "bottle". Phrases such as "pick up the cup" and "give me the
bottle" match because the object word is what counts.

To add a word, edit `DEFAULT_ALIASES` in
`src/assistant_interaction/assistant_interaction/voice_command.py`. The grammar is
built from that mapping (`grammar_json()`), so new aliases are recognised
without other changes. Check them with `scripts/check_voice.py`.

## Limits

- English small model only. Accuracy drops for accents and non-English speech.
- Use a quiet room and keep the phone near the speaker.
- Audio travels over Wi-Fi, so expect about 1 s of latency.
- Only the object classes above are recognised. Other words are ignored.
