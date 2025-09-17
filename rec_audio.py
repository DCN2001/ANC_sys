import serial
import wave
import struct

file_name = "./ESD_wav/F500.wav"
ser = serial.Serial('COM7', 921600, timeout=1)  

# WAV file
wav_file = wave.open(file_name, 'wb')
num_channels = 1        
sample_width = 2        
sample_rate = 16000     
wav_file.setnchannels(num_channels)
wav_file.setsampwidth(sample_width)
wav_file.setframerate(sample_rate)

print("Start recodring, press Ctrl+C to stop...")

try:
    while True:
        data = ser.read(2)  #  2 bits/time
        if len(data) == 2:
            # Write in the WAV file
            wav_file.writeframes(data)
            # Print value
            sample = struct.unpack('<h', data)[0]
            print(f"接收到的数据: {sample}")
        else:
            print("接收到的字节数不足:", len(data))
except KeyboardInterrupt:
    print("\n接收结束")
finally:
    ser.close()
    wav_file.close()
    print(f"Saving audio as {file_name}")