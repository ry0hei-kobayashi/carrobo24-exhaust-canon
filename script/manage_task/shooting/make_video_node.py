#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import numpy as np
import glob
import csv

from PIL import Image, ImageDraw, ImageFont
from progress.bar import IncrementalBar
import cv2

import rospy
import rospkg

GP_CLASS_NAMES = {
    0: "None",
    2: "Coffee can", 3: "Cheez-it", 4: "Sugar box", 5: "Tomato can",
    6: "Mustard", 7: "Tuna can", 8: "Choco box", 9: "Strawberry box",
    10: "Spam", 11: "Banana", 12: "Strawberry", 13: "Apple", 14: "Lemon",
    15: "Peach", 16: "Pear", 17: "Orange", 18: "Plum",
    19: "Pitcher base", 21: "Cleanser bottle", 22: "Spray bottle", 24: "Bowl",
    25: "Mug", 26: "Sponge", 29: "Plate", 30: "Fork", 31: "Spoon", 33: "Spatula",
    38: "Padlock", 40: "Marker", 50: "Clamp", 51: "Clamp", 52: "Clamp",
    53: "Soccer ball", 54: "Soft ball", 55: "Baseball", 56: "Tennis ball",
    57: "Racquetball", 58:"Golf ball", 59: "Chain", 61: "Foam brick",
    62: "Dice", 63: "Marbles", 65: "Cup",
    70: "Wood block", 71: "9-peg", 72: "Airplane", 73: "Lego",
    77: "Rubiks", 80: "Airplane", 81: "Airplane", 999: "Coke(Unknown)"
}

GP_CATEGORY_ID_RANGES = {
    "None": {"min": -1, "max": 0},
    "Food": {"min": 1, "max": 18},
    "Kitchen": {"min": 19, "max": 34},
    "Tool": {"min": 35, "max": 52},
    "Shape": {"min": 53, "max": 69},
    "Task": {"min": 70, "max": 82},
    "Food\n- Unknown": {"min": 999, "max": 1000}
}

GP_ORIENTATION_IDS = [30, 31, 40]

class MakeVideoNode:
    def __init__(self):
        rospy.init_node("make_video_node", anonymous=True)
        rospack = rospkg.RosPack()
        self._my_pkg_dir = rospack.get_path("compe_pkg")
        self._p_data_name = rospy.get_param("~data_name", None)
        self._p_video_time = rospy.get_param("~video_time", 10.0)
        
        if self._p_data_name == None:
            print("The parameter \"data_name\" is not set.")
            sys.exit(1)
        
        self._data_path = f"{self._my_pkg_dir}/io/images/{self._p_data_name}"
        self._font = f"{self._my_pkg_dir}/io/fonts/corpround-ver2-font/Corporate-Logo-Rounded.ttf"
    
    def cv2pil(self, image):
        new_image = image.copy()
        if new_image.ndim == 2:
            pass
        elif new_image.shape[2] == 3:
            new_image = cv2.cvtColor(new_image, cv2.COLOR_BGR2RGB)
        elif new_image.shape[2] == 4:
            new_image = cv2.cvtColor(new_image, cv2.COLOR_BGRA2RGBA)
        new_image = Image.fromarray(new_image)
        return new_image

    def pil2cv(self, image):
        new_image = np.array(image, dtype=np.uint8)
        if new_image.ndim == 2:
            pass
        elif new_image.shape[2] == 3:
            new_image = cv2.cvtColor(new_image, cv2.COLOR_RGB2BGR)
        elif new_image.shape[2] == 4:
            new_image = cv2.cvtColor(new_image, cv2.COLOR_RGBA2BGRA)
        return new_image

    def add_text_to_image(self, img, text, font_path, font_size, font_color, height, width):
        position = (width, height)
        font = ImageFont.truetype(font_path, font_size)

        draw = ImageDraw.Draw(img)
        draw.text(position, text, font_color, font=font)

        return img
    
    def run(self):
        video_time = self._p_video_time * 60.0  # [sec]

        print("Make video 0")
        files_0 = sorted(glob.glob(f"{self._data_path}/0/*.jpg"))
        files_1 = sorted(glob.glob(f"{self._data_path}/1/*.jpg"))

        bar = IncrementalBar('Converting', max=len(files_0))

        frame_rate = len(files_0) / video_time
        print(f"FPS:{frame_rate}")

        fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        video = cv2.VideoWriter(f'{self._data_path}/video.mp4', fourcc, frame_rate, (1920, 1080))

        team_name = ""
        data = []
        with open(f"{self._data_path}/data.csv") as f:
            reader = csv.reader(f)
            for i, row in enumerate(reader):
                if i == 0:
                    team_name = row[0]
                elif i != 1:
                    data.append(row)

        score = 0
        hit = False
        drop = False
        obj = "None"
        category = "None"
        data_i = 0
        for i in range(len(files_0)):
            cv_main = cv2.imread(files_0[i])
            cv_sub = cv2.imread(files_1[i])

            cv_sub_resized = cv2.resize(cv_sub, None, fx=0.4, fy=0.4)

            # add white-frame
            num_insert = 5
            frame_0 = np.ones((num_insert, cv_sub_resized.shape[1], 3), np.uint8) * 255
            array = np.insert(cv_sub_resized, 0, frame_0, axis=0)
            array = np.insert(array, array.shape[0], frame_0, axis=0)

            frame_1 = np.ones((array.shape[0], num_insert, 3), np.uint8) * 255
            array = np.insert(array, [0], frame_1, axis=1)
            array = np.insert(array, [array.shape[1]], frame_1, axis=1)

            # combine images
            cv_sub_resized_frame = np.array(array)
            offset_y, offset_x = cv_sub_resized_frame.shape[:2]
            cv_main[cv_main.shape[0] - offset_y:, cv_main.shape[1] - offset_x:] = cv_sub_resized_frame

            # draw data
            pil_main = self.cv2pil(cv_main)
            if int(data[data_i][0]) == i:
                # print("Data updated.")
                score = str(data[data_i][1])
                hit = True if int(data[data_i][2]) == 1 else False
                drop = True if int(data[data_i][3]) == 1 else False
                obj_id = int(data[data_i][4])
                obj = GP_CLASS_NAMES[obj_id]
                for key in GP_CATEGORY_ID_RANGES.keys():
                    if obj_id >= GP_CATEGORY_ID_RANGES[key]["min"] and obj_id <= GP_CATEGORY_ID_RANGES[key]["max"]:
                        category = key
                        break
                if obj_id in GP_ORIENTATION_IDS:
                    category += "\n- Orientation"
                if data_i < len(data) - 1:
                    data_i += 1

            # score
            draw = ImageDraw.Draw(pil_main)
            draw.rectangle((10, 13, 390, 90), fill=(150, 150, 150), outline=(255, 255, 255), width=3)
            pil_main = self.add_text_to_image(pil_main, "Score: " + str(score).zfill(3), self._font, 60, (255, 255, 255), 20, 30)

            # hit
            if hit:
                pil_main = self.add_text_to_image(pil_main, "HIT", self._font, 61, (0, 0, 0), 101, 31)
                pil_main = self.add_text_to_image(pil_main, "HIT", self._font, 60, (230, 0, 0), 100, 30)
            else:
                pil_main = self.add_text_to_image(pil_main, "HIT", self._font, 61, (100, 100, 100), 101, 31)
                pil_main = self.add_text_to_image(pil_main, "HIT", self._font, 60, (100, 100, 200), 100, 30)

            # drop
            if drop:
                pil_main = self.add_text_to_image(pil_main, "DROP", self._font, 61, (0, 0, 0), 171, 31)
                pil_main = self.add_text_to_image(pil_main, "DROP", self._font, 60, (230, 0, 0), 170, 30)
            else:
                pil_main = self.add_text_to_image(pil_main, "DROP", self._font, 61, (100, 100, 100), 171, 31)
                pil_main = self.add_text_to_image(pil_main, "DROP", self._font, 60, (100, 100, 200), 170, 30)

            # meta data
            draw.rectangle((1600, 13, 1910, 280), fill=(150, 150, 150), outline=(255, 255, 255), width=3)
            pil_main = self.add_text_to_image(pil_main, "Information", self._font, 40, (255, 255, 255), 20, 1620)
            pil_main = self.add_text_to_image(pil_main, "Object", self._font, 30, (255, 255, 255), 80, 1620)
            pil_main = self.add_text_to_image(pil_main, "- " + str(obj), self._font, 30, (255, 255, 255), 120, 1620)
            pil_main = self.add_text_to_image(pil_main, "Category", self._font, 30, (255, 255, 255), 165, 1620)
            pil_main = self.add_text_to_image(pil_main, "- " + str(category), self._font, 30, (255, 255, 255), 205, 1620)

            # Correct deposit
            points = None
            if "food" in category.lower():
                points = ((164*2, 227*2), (256*2, 228.5*2), (222*2, 338*2), (118*2, 338*2), (164*2, 227*2))
            elif "kitchen" in category.lower():
                points = ((206*2, 136*2), (265*2, 136*2), (246*2, 193*2), (183*2, 193*2), (206*2, 136*2))
            elif "tool" in category.lower():
                points = ((293*2, 12*2), (377*2, 22*2), (359*2, 110*2), (275*2, 100*2), (293*2, 12*2))
            elif "shape" in category.lower():
                points = ((295*2, 90*2), (359*2, 94*2), (345*2, 164*2), (279*2, 159*2), (295*2, 90*2))
            elif "task" in category.lower():
                points = ((118*2, 361*2), (220*2, 362*2), (191*2, 464*2), (79*2, 462*2), (118*2, 361*2))
            if points is not None:
                draw.line(points, fill=(255, 0, 0), width=9)
                for point in points:
                    draw.ellipse((point[0] - 4, point[1] - 4, point[0] + 4, point[1] + 4), fill=(255, 0, 0))

            if "orientation" in category.lower():
                points = ((195*2, 187*2), (235*2, 187*2), (225*2, 221*2), (185*2, 220*2), (195*2, 187*2))
                draw.line(points, fill=(255, 0, 0), width=9)
                for point in points:
                    draw.ellipse((point[0] - 4, point[1] - 4, point[0] + 4, point[1] + 4), fill=(255, 0, 0))

            if "unknown" in category.lower():
                points = ((78*2, 474*2), (188*2, 476*2), (168*2, 540*2), (49*2, 540*2), (78*2, 474*2))
                draw.line(points, fill=(255, 0, 0), width=9)
                for point in points:
                    draw.ellipse((point[0] - 4, point[1] - 4, point[0] + 4, point[1] + 4), fill=(255, 0, 0))

            cv_main = self.pil2cv(pil_main)

            # debug
            # cv_main_show = cv2.resize(cv_main, None, fx = 0.5, fy = 0.5)
            # cv2.imshow("cv_main", cv_main_show)
            # cv2.waitKey(0)

            bar.next()
            video.write(cv_main)

        video.release()
        bar.finish()
        print("Done.")

def main():
    node = MakeVideoNode()
    node.run()

if __name__ == "__main__":
    main()