import pyaudio
pa = pyaudio.PyAudio()
for i in range(pa.get_device_count()):
    d = pa.get_device_info_by_index(i)
    if d["maxInputChannels"] > 0:
        api = pa.get_host_api_info_by_index(d["hostApi"])["name"]
        print(f"[{i}] {d['name']}  ({api})")
pa.terminate()