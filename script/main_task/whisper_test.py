import numpy as np
import sounddevice as sd
import wave
import whisper

def audio_recog(file_name: str, wave_length: int, sample_rate: int) -> None:
    
    # 音声を録音し、指定されたファイル名で WAV フォーマットで保存します。

    # Parameters:
    # file_name (str): 保存するファイル名
    # wave_length (int): 録音する長さ（秒）
    # sample_rate (int): サンプリング周波数(Hz)
    
    # 録音開始
    print("録音開始...")
    data = sd.rec(int(wave_length * sample_rate), samplerate=sample_rate, channels=1, dtype='float32')
    sd.wait()  # 録音が完了するまで待機
    print("録音終了")

    # ノーマライズ。量子化ビット16bitで録音するので int16 の範囲で最大化する
    data = data / np.max(np.abs(data)) * np.iinfo(np.int16).max

    # float -> int
    data = data.astype(np.int16)

    # ファイル保存
    with wave.open(file_name, mode='wb') as wb:
        wb.setnchannels(1)  # モノラル
        wb.setsampwidth(2)  # 16bit = 2byte
        wb.setframerate(sample_rate)
        wb.writeframes(data.tobytes())  # バイト列に変換

    print(f"ファイル '{file_name}' に録音データを保存しました。")

def perform_voice_recognition(file_name: str) -> None:
    
    # 音声ファイルを読み込み、音声認識を実行し、結果をコンソールに表示します。

    # Parameters:
    # file_name (str): 音声ファイルのパス
    
    prompt_text = ""

    # モデルのロード
    model = whisper.load_model("medium")

    # 音声認識
    result = model.transcribe(file_name, language="ja", initial_prompt=prompt_text)

    # 認識結果の表示
    print(f"認識結果: {result['text']}")

if __name__ == "__main__":
    # ファイル名と録音パラメータの設定
    output_file_name = '/home/carrobo2024/ros_ws/src/group2/Day3/whisper/audio.wav'
    wave_length = 3  # 録音時間 (秒)
    sample_rate = 16_000  # サンプリングレート (Hz)

    # 録音
    audio_recog(output_file_name, wave_length, sample_rate)

    # 音声認識を実行
    perform_voice_recognition(output_file_name)