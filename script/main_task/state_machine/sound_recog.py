import numpy as np
import sounddevice as sd
import wave
import whisper
import smach
from tamlib.utils import Logger


class Voice_recog(smach.State, Logger):
    def __init__(self, outcomes, file_name: str):
        smach.State.__init__(self, outcomes=outcomes,
                            input_keys=['object', 'place', 'places'], 
                            output_keys=['object', 'place', 'places'])
        Logger.__init__(self)
        self.file_name =  file_name # 録音ファイルのパスを保存

    def execute(self, userdata):

        prompt_text = ""

        # モデルのロード
        model = whisper.load_model("medium")

        # 音声認識
        result = model.transcribe(self.file_name, language="ja", initial_prompt=prompt_text)

        if len(result['text'])<1:
            return loop
        # 認識結果の表示
        self.loginfo(f"認識結果: {result['text']}")

        # TODO:GPT API

        # userdata.object=
        # userdata.place=

        return "next"
    
    # if __name__ == "__main__":
    # # ファイル名と録音パラメータの設定
    #     output_file_name = '/home/carrobo2024/ros_ws/src/group2/Day3/whisper/audio.wav'
    #     wave_length = 5  # 録音時間 (秒)
    #     sample_rate = 16_000  # サンプリングレート (Hz)

    #     # 録音
    #     audio_recog(output_file_name, wave_length, sample_rate)

    #     # SMACH 状態の作成
    #     state = Voice_recog(outcomes=["next"], file_name=output_file_name)
    #     state.execute()