#!/usr/bin/env python3

import cv2
import csv
import numpy as np
import os
import random
import requests
import string

from random import randint
from PIL import Image, ImageFont, ImageDraw


def loadCrimesProfileCompetition():
    '''
    '''
    URL = "https://phas.ubc.ca/~miti/ENPH353/ENPH353Clues.csv"

    print("Loading clues ...")
    response = None
    while response is None:
        response = requests.get(URL)

    raw = response.text.split('\n')
    key_list   = raw[0].split(',')
    value_list = raw[1].split(',')

    clues = {}

    # We will save the clues to plates.csv
    # TODO Rename plates.csv to clues.csv
    with open(SCRIPT_PATH + "plates.csv", 'w') as plates_file:
        csvwriter = csv.writer(plates_file)

        for (key, value) in zip(key_list, value_list):
            clues[key] = value.upper()

        # save it to plates
        csvwriter.writerow([key, value])

    return clues

def loadCrimesProfileTrain():
    '''
    @brief returns a set of clues for one game and save them to plates.csv

    @retval clue dictionary of the form 
                [size:value, victim:value, ...
                 crime:value,time:value,
                 place:value,motive:value,
                 weapon:value,bandit:value]
    '''
    entries = {'SIZE': ["100", "10 GOOGLES", "314", "A PAIR", "BAKER DOZEN",
                    "COUNTLESS", "DOZEN", "FEW", "FIVE", "HALF DOZEN",
                    "LEGIONS", "MANY", "QUINTUPLETS", "RAYO10", "SINGLE",
                    "THREE", "TRIPLETS", "TWO", "UNCOUNTABLE", "ZEPTILLION"],
           'VICTIM': ["ALIENS", "ANTS", "BACTERIA", "BED BUGS", "BUNNIES",
                      "CITIZENS", "DINOSAURS", "FRODOS", "JEDIS", "KANGAROO",
                      "KOALAS", "PANDAS", "PARROTS", "PHYSICISTS", "QUOKKAS",
                      "ROBOTS", "RABBITS", "TOURISTS", "ZOMBIES"],
           'CRIME': ["ACCELERATE", "BITE", "CURSE", "DECELERATE", "DEFRAUD",
                     "DESTROY", "HEADBUT", "IRRADIATE", "LIE TO", "POKE",
                     "PUNCH", "PUSH", "SCARE", "STEAL", "STRIKE", "SWEAR",
                     "TELEPORT", "THINKING", "TICKLE", "TRANSMOGRIFY",
                     "TRESPASS"],
           'TIME': ["2023", "AUTUMN", "DAWN", "D DAY", "DUSK", "EONS AGO",
                    "JURASIC", "MIDNIGHT", "NOON", "Q DAY", "SPRING",
                    "SUMMER", "TOMORROW", "TWILIGHT", "WINTER", "YESTERDAY"],
           'PLACE': ["AMAZON", "ARCTIC", "BASEMENT", "BEACH", "BENU", "CAVE",
                     "CLASS", "EVEREST", "EXIT 8", "FIELD", "FOREST",
                     "HOSPITAL", "HOTEL", "JUNGLE", "MADAGASCAR", "MALL",
                     "MARS", "MINE", "MOON", "SEWERS", "SWITZERLAND",
                     "THE HOOD", "UNDERGROUND", "VILLAGE"],
           'MOTIVE': ["ACCIDENT", "BOREDOM", "CURIOSITY", "FAME", "FEAR",
                      "FOOLISHNESS", "GLAMOUR", "GLUTTONY", "GREED", "HATE",
                      "HASTE", "IGNORANCE", "IMPULSE", "LOVE", "LOATHING",
                      "PASSION", "PRIDE", "RAGE", "REVENGE", "REVOLT",
                      "SELF DEFENSE", "THRILL", "ZEALOUSNESS"],
           'WEAPON': ["ANTIMATTER", "BALOON", "CHEESE", "ELECTRON", "FIRE",
                      "FLASHLIGHT", "HIGH VOLTAGE", "HOLY GRENADE", "ICYCLE",
                      "KRYPTONITE", "NEUTRINOS", "PENCIL", "PLASMA",
                      "POLONIUM", "POSITRON", "POTATO GUN", "ROCKET", "ROPE",
                      "SHURIKEN", "SPONGE", "STICK", "TAMAGOCHI", "WATER",
                      "WRENCH"],
           'BANDIT': ["BARBIE", "BATMAN", "CAESAR", "CAO CAO", "EINSTEIN",
                      "GODZILA", "GOKU", "HANNIBAL", "L", "LENIN", "LUCIFER",
                      "LUIGI", "PIKACHU", "SATOSHI", "SHREK", "SAURON",
                      "THANOS", "TEMUJIN", "THE DEVIL", "ZELOS"]
           }
    
    clues = {}
    # We will save the clues to plates.csv
    # TODO Rename plates.csv to clues.csv
    with open(SCRIPT_PATH + "plates.csv", 'w') as plates_file:
        csvwriter = csv.writer(plates_file)

        for key in entries:
            # pick a random entry for the current clue (key)
            j = random.randint(0, len(entries[key])-1)
            random_value = entries[key][j]

            # add it to the current clue
            clues[key] = random_value

            # save it to plates
            csvwriter.writerow([key, random_value])

    return clues

# Find the path to this script
SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__)) + "/"
TEXTURE_PATH = '../media/materials/textures/'

banner_canvas = cv2.imread(SCRIPT_PATH+'clue_banner.png')
PLATE_HEIGHT = 600
PLATE_WIDTH = banner_canvas.shape[1]
IMG_DEPTH = 3

#clues = loadCrimesProfileTrain()
clues = loadCrimesProfileCompetition()

i = 0
for key, value in clues.items():
    entry = key + "," + value
    print(entry)

    # Generate plate

    # To use monospaced font for the license plate we need to use the PIL
    # package.
    # Convert into a PIL image (this is so we can use the monospaced fonts)
    blank_plate_pil = Image.fromarray(banner_canvas)
    # Get a drawing context
    draw = ImageDraw.Draw(blank_plate_pil)
    font_size = 90
    monospace = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf", 
                                    font_size)
    font_color = (255,0,0)
    draw.text((250, 30), key, font_color, font=monospace)
    draw.text((30, 250), value, font_color, font=monospace)
    # Convert back to OpenCV image and save
    populated_banner = np.array(blank_plate_pil)

    # Save image
    cv2.imwrite(os.path.join(SCRIPT_PATH+TEXTURE_PATH+"unlabelled/",
                                "plate_" + str(i) + ".png"), populated_banner)
    i += 1