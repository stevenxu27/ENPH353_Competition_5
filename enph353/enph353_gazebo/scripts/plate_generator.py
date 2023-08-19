#!/usr/bin/env python3

import cv2
import csv
import numpy as np
import os
import pyqrcode
import random
import string

from random import randint
from PIL import Image, ImageFont, ImageDraw

entries = {'size': ['TWO', 'A DOZEN', 'BUSY BEAVER 10'],
           'victim': ["PARROTS", "ROBOTS", "BACTERIA", "JEDIS"],
           'crime': ["STEAL", "TRESPASS", "LIE TO", "DESTROY"],
           'time': ["NOON", "MIDNIGHT", "DAWN", "DUSK", "64M YRS AGO"],
           'location': ["HOSPITAL", "MALL", "FOREST", "MOON"],
           'motive': ["GLUTTONY", "CURIOSITY", "IGNORANCE"],
           'weapon': ["A FISHING ROD", "ROCKET", "ANTIMATTER"],
           'criminal': ["EINSTEIN", "YODA", "SHREK", "LUIGI"]
           }

# Find the path to this script
SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__)) + "/"
TEXTURE_PATH = '../media/materials/textures/'

# write plates to plates.csv
with open(SCRIPT_PATH + "plates.csv", 'w') as plates_file:
    csvwriter = csv.writer(plates_file)

    i = 0
    for key in entries:
        # pick a random criminal
        j = random.randint(0, len(entries[key])-1)
        plate = entries[key][j]

        plate_str = plate
        print(plate_str)
        csvwriter.writerow([plate_str])

        # Generate plate
        blank_plate = cv2.imread(SCRIPT_PATH+'blank_plate.png')

        # To use monospaced font for the license plate we need to use the PIL
        # package.
        # Convert into a PIL image (this is so we can use the monospaced fonts)
        blank_plate_pil = Image.fromarray(blank_plate)
        # Get a drawing context
        draw = ImageDraw.Draw(blank_plate_pil)
        monospace = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf", 200)
        draw.text((48, 105),plate_str, (255,0,0), font=monospace)
        # Convert back to OpenCV image and save
        blank_plate = np.array(blank_plate_pil)

        # Create parking spot label
        s = key
        parking_spot = 255 * np.ones(shape=[600, 600, 3], dtype=np.uint8)
        parking_spot_pil = Image.fromarray(parking_spot)
        draw = ImageDraw.Draw(parking_spot_pil)
        monospace = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf", 200)
        draw.text((48, 105), s, (255,0,0), font=monospace)
        parking_spot = np.array(parking_spot_pil)
        spot_w_plate = np.concatenate((parking_spot, blank_plate), axis=0)

         # Merge images and save
        unlabelled = np.concatenate((255 * np.ones(shape=[600, 600, 3],
                                    dtype=np.uint8), spot_w_plate), axis=0)
        cv2.imwrite(os.path.join(SCRIPT_PATH+TEXTURE_PATH+"unlabelled/",
                                 "plate_" + str(i) + ".png"), unlabelled)
        i += 1

'''
with open(path + "plates.csv", 'w') as plates_file:
    csvwriter = csv.writer(plates_file)

    for i in range(0, 16):

        # Pick two random letters
        plate_alpha = ""
        for _ in range(0, 2):
            plate_alpha += (random.choice(string.ascii_uppercase))
        num = randint(0, 99)

        # Pick two random numbers
        plate_num = "{:02d}".format(num)

        # Save plate to file
        csvwriter.writerow([plate_alpha+plate_num])

        # Write plate to image
        blank_plate = cv2.imread(path+'blank_plate.png')

        # To use monospaced font for the license plate we need to use the PIL
        # package.
        # Convert into a PIL image (this is so we can use the monospaced fonts)
        blank_plate_pil = Image.fromarray(blank_plate)
        # Get a drawing context
        draw = ImageDraw.Draw(blank_plate_pil)
        monospace = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf", 200)
        draw.text((48, 105),plate_alpha + " " + plate_num, (255,0,0), font=monospace)
        # Convert back to OpenCV image and save
        blank_plate = np.array(blank_plate_pil)

        # cv2.putText(blank_plate,
        #             plate_alpha + " " + plate_num, (45, 360),
        #             cv2.FONT_HERSHEY_PLAIN, 11, (255, 0, 0), 7, cv2.LINE_AA)

        # Create QR code image
        spot_name = "P" + str(i)
        qr = pyqrcode.create(spot_name+"_" + plate_alpha + plate_num)
        qrname = path + "QRCode_" + str(i) + ".png"
        qr.png(qrname, scale=20)
        QR_img = cv2.imread(qrname)
        QR_img = cv2.resize(QR_img, (600, 600), interpolation=cv2.INTER_AREA)

        # Create parking spot label
        s = "P" + str(i+1)
        parking_spot = 255 * np.ones(shape=[600, 600, 3], dtype=np.uint8)
        cv2.putText(parking_spot, s, (30, 450), cv2.FONT_HERSHEY_PLAIN, 28,
                    (0, 0, 0), 30, cv2.LINE_AA)
        spot_w_plate = np.concatenate((parking_spot, blank_plate), axis=0)

        # Merge labelled or unlabelled images and save
        labelled = np.concatenate((QR_img, spot_w_plate), axis=0)
        unlabelled = np.concatenate((255 * np.ones(shape=[600, 600, 3],
                                    dtype=np.uint8), spot_w_plate), axis=0)
        cv2.imwrite(os.path.join(path + texture_path + "labelled/",
                                 "plate_" + str(i) + ".png"), labelled)
        cv2.imwrite(os.path.join(path+texture_path+"unlabelled/",
                                 "plate_" + str(i) + ".png"), unlabelled)
'''