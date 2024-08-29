#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import wave
import whisper
import smach
from tamlib.utils import Logger
import openai
import rospy
import re


class Voice_recog(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes,
                            input_keys=['object', 'place', 'places'], 
                            output_keys=['object', 'place', 'places'])
        Logger.__init__(self)
        self.file_name =  '/home/carrobo2024/ros_ws/src/compe_pkg/io/audio/shell_orange_sato.wav'

        # APIキーの設定
        openai.api_key =  # ここにあなたのAPIキーを入力

    def execute(self, userdata):
        print("sound")

        prompt_text = ""

        # モデルのロード
        model = whisper.load_model("small")

        # 音声認識
        result = model.transcribe(self.file_name, language="ja", initial_prompt=prompt_text)

        if len(result['text'])<1:
            return loop
        # 認識結果の表示
        rospy.loginfo(f"認識結果: {result['text']}")


        # プロンプトの設定
        prompt = f"次の文からオブジェクト名と置いてある家具名を英語で抽出してください。:{result['text']}出力形式は、'オブジェクト名:[オブジェクト名を英語で出力], 場所:[家具名を英語で出力]'。"

        # 正しいモデル名を使用して応答を生成
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",  # 正しいモデル名
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.7
        )

        text = response['choices'][0]['message']['content'].strip()
        rospy.loginfo(text)

        # 「オブジェクト名:」に続く部分を取り出す
        object_match = re.search(r'オブジェクト名:(.*?)(,|$)', text)

        # 「場所:」に続く部分を取り出す
        location_match = re.search(r'場所:(.*?)(,|$)', text)
        if object_match:
            userdata.object = object_match.group(1).strip()
            rospy.loginfo(userdata.object)

        if location_match:
            userdata.place = location_match.group(1).strip()
            rospy.loginfo(userdata.place)

        return "next"
    
if __name__ == "__main__":
    rospy.init_node('state_machine_nav')

    sm = smach.StateMachine(outcomes=['exit']) 
    with sm:
        smach.StateMachine.add('DEBUG',  Voice_recog(outcomes=["next"]),
                               transitions={'next': 'exit'})
    sm.execute()
