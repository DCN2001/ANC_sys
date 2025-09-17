import numpy as np
import librosa
import mir_eval

def calculate_sdr(input_audio_path):
    # 讀取輸入音頻
    input_audio, sr = librosa.load(input_audio_path, sr=None, mono=True)

    # 目標參考音頻 (全零信號)
    reference_audio = np.zeros_like(input_audio)
    
    # 確保長度一致
    min_length = min(len(input_audio), len(reference_audio))
    input_audio = input_audio[:min_length]
    reference_audio = reference_audio[:min_length]
    
    # 計算 SDR
    sdr, _, _, _ = mir_eval.separation.bss_eval_sources(
        np.expand_dims(reference_audio, axis=0),  # 全零參考信號
        np.expand_dims(input_audio, axis=0)      # 輸入音頻
    )
    
    return sdr[0]

# 使用範例
input_audio_path = "./ESD_wav/testing.wav"  # 替換為你的輸入音頻路徑
sdr = calculate_sdr(input_audio_path)
print(f"SDR: {sdr:.2f} dB")
