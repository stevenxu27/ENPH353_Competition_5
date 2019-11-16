#!/usr/bin/env python
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import (Qt, pyqtSignal)
from std_msgs.msg import String
from python_qt_binding import loadUi

import csv
import os
import rospy
import sys

NUM_LOCATIONS = 8

class Window(QtWidgets.QMainWindow):
    license_plate_signal = pyqtSignal(str)

    def __init__(self):
        super(Window, self).__init__()
        loadUi("./score_tracker.ui", self)

        # Populate tables
        LICENSE_PLATE_FILE = '/../../enph353_gazebo/scripts/plates.csv'
        SCRIPT_FILE_PATH = os.path.dirname(os.path.realpath(__file__))
        with open(SCRIPT_FILE_PATH + LICENSE_PLATE_FILE, "r") as plate_file:
            platereader = csv.reader(plate_file)
            i=0
            for row in platereader:
                if i < NUM_LOCATIONS:
                    self.license_scores_QTW.item(i, 1).setText(row[0])
                    self.log_msg("Position {}: {}".format(i+1, row[0]))
                else:
                    break
                i += 1

        # Connect widgets
        self.penalty_vehicle_QPB.clicked.connect(self.penalty_vehicle)
        self.penalty_pedestrian_QPB.clicked.connect(self.penalty_pedestrian)
        self.penalty_track_QPB.clicked.connect(self.penalty_track)

        self.license_plate_signal.connect(self.update_license_plates)

        self.sub = rospy.Subscriber("license_plate", String, 
                                    self.licensePlate_callback)
        rospy.init_node('my_listener')


    def penalty_pedestrian(self):
        numEvents       = int(self.penalties_scores_QTW.item(1, 1).text()) + 1
        penaltyPerEvent = int(self.penalties_scores_QTW.item(1, 2).text())
        penaltyTotal    = numEvents * penaltyPerEvent
        self.penalties_scores_QTW.item(1, 1).setText(str(numEvents))
        self.penalties_scores_QTW.item(1, 3).setText(str(penaltyTotal))

        self.log_msg("Penalty: pedestrian collision: -10 pts")
        self.update_penalty_total()


    def penalty_track(self):
        numEvents       = int(self.penalties_scores_QTW.item(2, 1).text()) + 1
        penaltyPerEvent = int(self.penalties_scores_QTW.item(2, 2).text())
        penaltyTotal    = numEvents * penaltyPerEvent
        self.penalties_scores_QTW.item(2, 1).setText(str(numEvents))
        self.penalties_scores_QTW.item(2, 3).setText(str(penaltyTotal))

        self.log_msg("Penalty: track limit: -2 pts")
        self.update_penalty_total()


    def penalty_vehicle(self):
        numEvents       = int(self.penalties_scores_QTW.item(0, 1).text()) + 1
        penaltyPerEvent = int(self.penalties_scores_QTW.item(0, 2).text())
        penaltyTotal    = numEvents * penaltyPerEvent
        self.penalties_scores_QTW.item(0, 1).setText(str(numEvents))
        self.penalties_scores_QTW.item(0, 3).setText(str(penaltyTotal))

        self.log_msg("Penalty: vehicle collision: -5 pts")
        self.update_penalty_total()


    def update_license_total(self):
        licenseTotal = 0
        for i in range(NUM_LOCATIONS):
            licenseTotal += int(self.license_scores_QTW.item(i, 3).text())

        self.license_total_value_QL.setText(str(licenseTotal))

        penaltyTotal = int(self.penalties_total_value_QL.text())
        teamTotal = penaltyTotal + licenseTotal
        self.total_score_value_QL.setText(str(teamTotal))
        self.log_msg("Team total: {} pts".format(str(teamTotal)))


    def update_penalty_total(self):
        penaltyVehicle    = int(self.penalties_scores_QTW.item(0, 3).text())
        penaltyPedestrian = int(self.penalties_scores_QTW.item(1, 3).text())
        penaltyTrack      = int(self.penalties_scores_QTW.item(2, 3).text())

        penaltyTotal = penaltyVehicle + penaltyPedestrian + penaltyTrack
        self.penalties_total_value_QL.setText(str(penaltyTotal))

        licenseTotal = int(self.license_total_value_QL.text())
        teamTotal = penaltyTotal + licenseTotal
        self.total_score_value_QL.setText(str(teamTotal))
        self.log_msg("Team total: {} pts".format(str(teamTotal)))


    def log_msg(self, message):
        self.comms_log_QTE.append(message)


    def licensePlate_callback(self, data):
        self.license_plate_signal.emit(str(data.data))


    def update_license_plates(self, license_string):
        self.log_msg("Callback called: {}".format(license_string))

        teamID, teamPswd, plateLocation, plateID = str(license_string).split(',')
        plateLocation = int(plateLocation)

        self.team_ID_value_QL.setText(teamID)

        self.license_scores_QTW.item(plateLocation-1, 2).setText(plateID)
        gndTruth = str(self.license_scores_QTW.item(plateLocation-1, 1).text())

        if gndTruth == plateID:
            self.license_scores_QTW.item(plateLocation-1, 3).setText(str(5))
            self.log_msg("Awarded: {} pts".format(5))
        else:
            self.license_scores_QTW.item(plateLocation-1, 3).setText(str(-5))
            self.log_msg("Awarded: {} pts".format(-5))

        self.update_license_total()

if __name__ == "__main__":
    # monitor which topics get subscribed by the user
    # emphasize the only topics teams are allowed to subscribe to
    # teamname,teampasswd,location,platid

    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()

    sys.exit(app.exec_())