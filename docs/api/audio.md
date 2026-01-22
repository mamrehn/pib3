# Audio System

The pib3 audio system provides unified audio playback, text-to-speech (TTS), and recording capabilities that work seamlessly across both the real robot and Webots simulation.

## Overview

The audio system consists of several key components:

- **Audio Playback**: Play raw audio data or WAV files on local speakers, robot speakers, or both
- **Text-to-Speech**: Synthesize speech using Piper TTS with German and English voices
- **Audio Recording**: Stream audio from the robot's microphone array
- **Output Routing**: Control where audio plays using the `AudioOutput` enum

## Audio Format

All audio in the pib3 system uses a standardized format:

- **Sample rate**: 16000 Hz (16kHz)
- **Channels**: 1 (Mono)
- **Bit depth**: 16-bit signed integers (int16)

## Quick Start

```python
from pib3 import Robot, AudioOutput
import numpy as np

with Robot(host="172.26.34.149") as robot:
    # Text-to-speech (German speech model and on the robot device by default)
    robot.speak("Hallo! Ich bin pib, dein freundlicher Roboter.")
    
    # Play on local speakers only
    robot.speak("Testing local audio", output=AudioOutput.LOCAL)
    
    # Play on both robot and laptop
    robot.speak("Playing everywhere!", output=AudioOutput.LOCAL_AND_ROBOT)
    
    # Play a WAV file (on the robot device)
    robot.play_file("sound.wav", output=AudioOutput.ROBOT)
    
    # Generate and play a tone (on the robot device)
    tone = (np.sin(2 * np.pi * 440 * np.linspace(0, 1, 16000)) * 16000).astype(np.int16)
    robot.play_audio(tone, output=AudioOutput.ROBOT)
```

## AudioOutput Enum

The `AudioOutput` enum controls where audio is played:

```python
from pib3 import AudioOutput

# Available options:
AudioOutput.LOCAL              # Play on local machine (laptop) speakers only
AudioOutput.ROBOT              # Play on robot speakers only
AudioOutput.LOCAL_AND_ROBOT    # Play on both simultaneously
```

### Backend-Specific Behavior

- **Real Robot**: Default is `AudioOutput.ROBOT`
  - `LOCAL`: Plays on your laptop/computer
  - `ROBOT`: Sends audio to robot via `/audio_playback` ROS topic
  - `LOCAL_AND_ROBOT`: Plays on both (best-effort synchronization)

- **Webots Simulation**: Default is `AudioOutput.LOCAL`
  - `LOCAL`: Plays on your laptop/computer
  - `ROBOT`: Automatically redirects to `LOCAL` (no robot speakers in simulation)
  - `LOCAL_AND_ROBOT`: Automatically redirects to `LOCAL` (no duplication)

## API Reference

### Playback Methods

#### `play_audio()`

Play raw audio data.

```python
robot.play_audio(
    data: Union[bytes, np.ndarray, List[int]],
    sample_rate: int = 16000,
    output: Optional[AudioOutput] = None,
    block: bool = True
) -> bool
```

**Parameters:**

- `data`: Audio data as bytes, numpy array (int16), or list of int16 samples
- `sample_rate`: Sample rate in Hz (default: 16000)
- `output`: Playback destination (LOCAL, ROBOT, LOCAL_AND_ROBOT). If None, uses backend default
- `block`: If True, wait for playback to complete before returning

**Returns:** `True` if playback succeeded

**Example:**

```python
import numpy as np

# Generate a 440Hz tone for 1 second
sample_rate = 16000
t = np.linspace(0, 1.0, sample_rate, dtype=np.float32)
tone = (np.sin(2 * np.pi * 440 * t) * 16000).astype(np.int16)

# Play on robot
robot.play_audio(tone, output=AudioOutput.ROBOT)

# Play on both robot and laptop
robot.play_audio(tone, output=AudioOutput.LOCAL_AND_ROBOT)
```

---

#### `play_file()`

Play audio from a WAV file.

```python
robot.play_file(
    filepath: Union[str, Path],
    output: Optional[AudioOutput] = None,
    block: bool = True
) -> bool
```

**Parameters:**

- `filepath`: Path to WAV file
- `output`: Playback destination. If None, uses backend default
- `block`: If True, wait for playback to complete

**Returns:** `True` if playback succeeded

**WAV File Format:**

The function accepts WAV files with the following formats (automatic conversion is applied):

- **Sample Rate**: Any sample rate (automatically resampled to 16000 Hz)
- **Channels**: Mono or Stereo (stereo is automatically converted to mono by averaging channels)
- **Bit Depth**: 
  - 8-bit (unsigned, converted to 16-bit signed)
  - 16-bit (signed, preferred format)
  - 32-bit (signed, converted to 16-bit signed)

> **Note:** While various formats are supported, for best performance use WAV files with:
> - **16000 Hz sample rate**
> - **Mono (1 channel)**
> - **16-bit signed PCM**

**Example:**

```python
# Play a sound effect on robot
robot.play_file("sounds/beep.wav", output=AudioOutput.ROBOT)

# Play background music locally while robot works
robot.play_file("music.wav", output=AudioOutput.LOCAL, block=False)

# Any standard WAV format works (automatically converted)
robot.play_file("44100hz_stereo.wav")  # Resampled to 16kHz mono
robot.play_file("8bit_audio.wav")      # Converted to 16-bit
```

---

### Text-to-Speech

#### `speak()`

Synthesize and play text-to-speech using Piper TTS.

```python
robot.speak(
    text: str,
    output: Optional[AudioOutput] = None,
    voice: Optional[str] = None,
    block: bool = True
) -> bool
```

**Parameters:**

- `text`: Text to speak
- `output`: Playback destination. If None, uses backend default
- `voice`: Piper voice model name (default: `"de_DE-thorsten-high"` for German)
- `block`: If True, wait for speech to complete

**Returns:** `True` if speech was played successfully

**Available Voices:**

- `"de_DE-thorsten-high"` - German (Thorsten voice, high quality) - **Default**
- `"en_US-lessac-medium"` - English (Lessac voice, medium quality)
- Other Piper voices from [rhasspy/piper-voices](https://huggingface.co/rhasspy/piper-voices)

Voice models are automatically downloaded on first use and cached in `~/.cache/pib3/piper_models/`.

**Example:**

```python
# German (default)
robot.speak("Hallo! Wie geht es dir?")

# English
robot.speak("Hello! How are you?", voice="en_US-lessac-medium")

# Play on both robot and laptop
robot.speak("Wichtige Nachricht!", output=AudioOutput.LOCAL_AND_ROBOT)

# Fire and forget (don't wait)
robot.speak("Ich arbeite...", block=False)
```

---

### Audio Recording

#### `subscribe_audio_stream()`

Subscribe to audio stream from the robot's microphone array.

```python
robot.subscribe_audio_stream(
    callback: Callable[[List[int]], None]
) -> roslibpy.Topic
```

**Parameters:**

- `callback`: Function called with list of int16 audio samples for each chunk (~1024 samples)

**Returns:** Topic object (call `.unsubscribe()` to stop streaming)

**Audio Format:**
- Sample rate: 16000 Hz
- Channels: 1 (Mono)
- Bit depth: 16-bit signed integers
- Chunk size: ~1024 samples per message

**Example:**

```python
import numpy as np
from pib3 import AudioStreamReceiver

# Using AudioStreamReceiver helper
receiver = AudioStreamReceiver()
sub = robot.subscribe_audio_stream(receiver.on_audio)

# Record for 10 seconds
import time
time.sleep(10)
sub.unsubscribe()

# Save to WAV file
receiver.save_wav("recording.wav")

# Or get as numpy array
audio_data = receiver.get_audio()
print(f"Recorded {receiver.duration:.1f} seconds")
```

**Manual callback example:**

```python
audio_buffer = []

def on_audio(samples):
    audio_buffer.extend(samples)
    print(f"Received {len(samples)} samples")

sub = robot.subscribe_audio_stream(on_audio)
time.sleep(5)
sub.unsubscribe()

# Convert to numpy array
audio = np.array(audio_buffer, dtype=np.int16)
```

---

### Helper Classes

#### `AudioStreamReceiver`

Helper class for buffering and saving audio from the robot's microphone.

```python
from pib3 import AudioStreamReceiver

receiver = AudioStreamReceiver(
    sample_rate: int = 16000,
    max_buffer_seconds: float = 300.0
)
```

**Methods:**

- `on_audio(samples)` - Callback for incoming audio (pass to `subscribe_audio_stream()`)
- `get_audio()` - Get buffered audio as numpy array (int16)
- `get_audio_float()` - Get buffered audio as normalized float32 in range [-1, 1]
- `clear()` - Clear the audio buffer
- `save_wav(filepath)` - Save buffered audio to WAV file

**Properties:**

- `duration` - Duration of buffered audio in seconds
- `sample_count` - Number of samples in buffer

**Example:**

```python
receiver = AudioStreamReceiver()
sub = robot.subscribe_audio_stream(receiver.on_audio)

time.sleep(10)
sub.unsubscribe()

print(f"Recorded {receiver.duration:.1f}s ({receiver.sample_count} samples)")
receiver.save_wav("recording.wav")
```

---

## ROS Topics

The audio system uses the following ROS topics for communication with the robot:

### `/audio_playback`

**Type:** `std_msgs/msg/Int16MultiArray`  
**Direction:** Client → Robot  
**Purpose:** Send audio data to robot for playback through speakers

**Message Format:**
```python
{
    "layout": {"dim": [], "data_offset": 0},
    "data": [int16, int16, ...]  # Audio samples
}
```

This topic is used internally by `RobotAudioPlayer` when `output=AudioOutput.ROBOT`.

---

### `/audio_stream`

**Type:** `std_msgs/msg/Int16MultiArray`  
**Direction:** Robot → Client  
**Purpose:** Stream audio from robot's microphone array

**Message Format:**
```python
{
    "layout": {...},
    "data": [int16, int16, ...]  # Audio samples (~1024 per message)
}
```

Subscribe to this topic using `robot.subscribe_audio_stream()`.

---

## Installation Requirements

### Core Audio (Required)

```bash
pip install pib3
```

### Local Playback (Optional but Recommended)

For playing audio on your local machine:

```bash
# Linux (install system dependency first)
sudo apt-get install libasound2-dev
pip install simpleaudio

# macOS
pip install simpleaudio

# Windows (may need Visual C++ Build Tools)
pip install simpleaudio
```

### Text-to-Speech (Optional)

For TTS functionality:

```bash
pip install piper-tts
```

---

## Platform-Specific Notes

### Linux

Local audio playback requires ALSA development libraries:

```bash
sudo apt-get install libasound2-dev
pip install simpleaudio
```

### macOS

No additional system dependencies required:

```bash
pip install simpleaudio
```

### Windows

May require Microsoft Visual C++ Build Tools for `simpleaudio`:

1. Download from: https://visualstudio.microsoft.com/visual-cpp-build-tools/
2. Install the build tools
3. `pip install simpleaudio`

---

## Advanced Usage

### Custom Audio Processing

```python
import numpy as np
from pib3 import Robot, AudioOutput

with Robot(host="172.26.34.149") as robot:
    # Generate custom waveform
    sample_rate = 16000
    duration = 2.0
    t = np.linspace(0, duration, int(sample_rate * duration))
    
    # Frequency sweep from 200Hz to 800Hz
    freq = np.linspace(200, 800, len(t))
    audio = (np.sin(2 * np.pi * freq * t) * 16000).astype(np.int16)
    
    # Apply fade in/out
    fade_samples = int(0.1 * sample_rate)
    audio[:fade_samples] *= np.linspace(0, 1, fade_samples)
    audio[-fade_samples:] *= np.linspace(1, 0, fade_samples)
    
    robot.play_audio(audio, output=AudioOutput.ROBOT)
```

### Recording and Playback

```python
from pib3 import Robot, AudioStreamReceiver, AudioOutput
import time

with Robot(host="172.26.34.149") as robot:
    # Record audio from robot
    receiver = AudioStreamReceiver()
    sub = robot.subscribe_audio_stream(receiver.on_audio)
    
    robot.speak("Ich höre jetzt zu. Bitte sprechen Sie.")
    time.sleep(5)
    sub.unsubscribe()
    
    # Play back the recording
    audio = receiver.get_audio()
    robot.speak("Das haben Sie gesagt:")
    robot.play_audio(audio, output=AudioOutput.ROBOT)
```

### Multi-Language TTS

```python
# Switch between languages
robot.speak("Hallo! Ich spreche Deutsch.", voice="de_DE-thorsten-high")
robot.speak("Hello! I speak English.", voice="en_US-lessac-medium")
```

---

## Examples

Complete example scripts are available in the `examples/` directory:

- [`audio_example_simple.py`](file:///home/amr/repositories/b3_pib_inverse_kinematic_pip_package/examples/audio_example_simple.py) - Basic audio playback and TTS
- [`audio_example_advanced.py`](file:///home/amr/repositories/b3_pib_inverse_kinematic_pip_package/examples/audio_example_advanced.py) - Recording, TTS, and advanced features

Run them with:

```bash
# Real robot
python examples/audio_example_simple.py --host 172.26.34.149

# Webots simulation
python examples/audio_example_simple.py --webots

# Specify output destination
python examples/audio_example_simple.py --output local
python examples/audio_example_simple.py --output robot
python examples/audio_example_simple.py --output both
```

---

## Troubleshooting

### No audio on local machine

**Problem:** `simpleaudio not available` warning

**Solution:**
```bash
# Linux
sudo apt-get install libasound2-dev
pip install simpleaudio

# macOS/Windows
pip install simpleaudio
```

---

### TTS not working

**Problem:** `piper-tts is required for text-to-speech`

**Solution:**
```bash
pip install piper-tts
```

Voice models are downloaded automatically on first use (~20-50MB per voice).

---

### Audio not playing on robot

**Problem:** Audio plays locally but not on robot

**Checklist:**
1. Verify robot connection: `robot.is_connected` should be `True`
2. Check output setting: Use `output=AudioOutput.ROBOT`
3. Verify rosbridge is running on robot
4. Check robot's audio system is functional

---

### Audio quality issues

**Problem:** Distorted or choppy audio

**Solutions:**
- Ensure audio is 16kHz, mono, int16 format
- Check network latency (for robot playback)
- Reduce audio chunk size for streaming
- Verify sample rate matches (16000 Hz)

---

## See Also

- [Backends API](backends/base.md) - Base backend interface
- [Robot Backend](backends/robot.md) - Real robot backend
- [Webots Backend](backends/webots.md) - Simulation backend
- [Examples](file:///home/amr/repositories/b3_pib_inverse_kinematic_pip_package/examples/) - Complete example scripts
