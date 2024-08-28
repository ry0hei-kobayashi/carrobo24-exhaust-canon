import numpy as np
import sounddevice as sd
import wave
import whisper
import smach
from tamlib.utils import Logger

def audio_recog(file_name: str, wave_length: int, sample_rate: int) -> None:
    """
    音声を録音し、指定されたファイル名で WAV フォーマットで保存します。

    Parameters:
    file_name (str): 保存するファイル名
    wave_length (int): 録音する長さ（秒）
    sample_rate (int): サンプリング周波数(Hz)
    """
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


class Voice_recog(smach.State, Logger):
    def __init__(self, outcomes, file_name: str):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self)
        self.file_name =  file_name # 録音ファイルのパスを保存

    def execute(self):

        prompt_text = ""

        # モデルのロード
        model = whisper.load_model("medium")

        # 音声認識
        result = model.transcribe(self.file_name, language="ja", initial_prompt=prompt_text)

        # 認識結果の表示
        self.loginfo(f"認識結果: {result['text']}")

        return "next"
    
    if __name__ == "__main__":
    # ファイル名と録音パラメータの設定
        output_file_name = '/home/carrobo2024/ros_ws/src/group2/Day3/whisper/audio.wav'
        wave_length = 5  # 録音時間 (秒)
        sample_rate = 16_000  # サンプリングレート (Hz)

        # 録音
        audio_recog(output_file_name, wave_length, sample_rate)

        # SMACH 状態の作成
        state = Voice_recog(outcomes=["next"], file_name=output_file_name)
        state.execute()