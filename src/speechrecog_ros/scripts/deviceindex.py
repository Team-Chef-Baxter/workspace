import pyaudio

def list_audio_devices():
    # Initialize PyAudio
    p = pyaudio.PyAudio()
    
    print("Available Audio Devices:")
    
    # Iterate through the available devices
    for i in range(p.get_device_count()):
        device_info = p.get_device_info_by_index(i)
        name = device_info.get("name", "")
        max_input_channels = device_info.get("maxInputChannels", 0)
        
        # Display devices with input channels (microphones)
        if max_input_channels > 0:
            print(f"Index {i}: {name}")
            # Check for 'Logitech' or similar in the device name
            if "logi" in name.lower() or "logitech" in name.lower():
                print(f"  >> Likely Logitech device detected at index {i}")
    
    # Terminate PyAudio
    p.terminate()

if __name__ == "__main__":
    list_audio_devices()
