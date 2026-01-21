import abc
from typing import Union, List, Optional
import numpy as np

try:
    import simpleaudio as sa
except ImportError:
    sa = None

try:
    import roslibpy
except ImportError:
    roslibpy = None


class AudioBackend(abc.ABC):
    """Abstract base class for audio backends."""

    @abc.abstractmethod
    def play(self, data: Union[bytes, np.ndarray, List[int]], sample_rate: int = 16000) -> bool:
        """
        Play audio data.

        Args:
            data: Audio data. Can be:
                - bytes: Raw 16-bit PCM bytes.
                - np.ndarray: NumPy array of int16 values.
                - List[int]: List of int16 values.
            sample_rate: Sample rate in Hz. Default: 16000.

        Returns:
            True if playback started successfully, False otherwise.
        """
        pass

    @abc.abstractmethod
    def stop(self) -> None:
        """Stop current playback."""
        pass


class NoOpAudioBackend(AudioBackend):
    """Fallback backend that does nothing."""

    def play(self, data: Union[bytes, np.ndarray, List[int]], sample_rate: int = 16000) -> bool:
        return False

    def stop(self) -> None:
        pass


class SystemAudioBackend(AudioBackend):
    """
    Plays audio locally using simpleaudio (cross-platform).
    """

    def __init__(self):
        if sa is None:
            raise ImportError(
                "simpleaudio is required for SystemAudioBackend. "
                "Install it with: pip install simpleaudio"
            )
        self.play_obj = None

    def play(self, data: Union[bytes, np.ndarray, List[int]], sample_rate: int = 16000) -> bool:
        # Convert to bytes if needed
        if isinstance(data, np.ndarray):
            audio_data = data.astype(np.int16).tobytes()
        elif isinstance(data, list):
            audio_data = np.array(data, dtype=np.int16).tobytes()
        elif isinstance(data, bytes):
            audio_data = data
        else:
            raise ValueError(f"Unsupported audio data type: {type(data)}")

        self.stop()  # Stop any previous playback

        try:
            # simpleaudio expects raw PCM data
            # 1 channel (mono), 2 bytes per sample (16-bit)
            self.play_obj = sa.play_buffer(audio_data, 1, 2, sample_rate)
            return True
        except Exception as e:
            print(f"Error playing audio: {e}")
            return False

    def stop(self) -> None:
        if self.play_obj and self.play_obj.is_playing():
            self.play_obj.stop()
            self.play_obj = None


class ROSAudioBackend(AudioBackend):
    """
    Streams audio to a ROS topic.
    Target topic: /audio_stream
    Message type: std_msgs/Int16MultiArray
    """

    def __init__(self, client):
        """
        Args:
            client: roslibpy.Ros instance.
        """
        if roslibpy is None:
            raise ImportError("roslibpy is required for ROSAudioBackend.")
            
        self.client = client
        self.topic = roslibpy.Topic(client, '/audio_stream', 'std_msgs/Int16MultiArray')

    def play(self, data: Union[bytes, np.ndarray, List[int]], sample_rate: int = 16000) -> bool:
        if not self.client.is_connected:
            return False

        # Convert to List[int] for Int16MultiArray
        if isinstance(data, bytes):
            # Bytes to int16 list
            dt = np.dtype(np.int16)
            dt = dt.newbyteorder('<') # Assume little-endian
            int_data = np.frombuffer(data, dtype=dt).tolist()
        elif isinstance(data, np.ndarray):
            int_data = data.astype(np.int16).tolist()
        elif isinstance(data, list):
            int_data = [int(x) for x in data]
        else:
            raise ValueError(f"Unsupported audio data type: {type(data)}")

        # Construct Int16MultiArray message
        # Layout is optional but good practice. We can omit for simplicity if receiver handles it.
        # std_msgs/Int16MultiArray definition:
        #   MultiArrayLayout layout
        #   int16[] data
        
        msg = {
            "layout": {
                "dim": [],
                "data_offset": 0
            },
            "data": int_data
        }

        try:
            self.topic.publish(roslibpy.Message(msg))
            return True
        except Exception as e:
            print(f"Error streaming audio to ROS: {e}")
            return False

    def stop(self) -> None:
        # ROS topic approach is fire-and-forget for chunks. 
        # A 'stop' might imply sending an empty message or specific control command,
        # but for raw stream, we just stop publishing.
        pass
