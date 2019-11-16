#!/usr/bin/env python
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from std_msgs.msg import String

import csv
import os
import rospy
import sys

NUM_LOCATIONS = 8

class Window(QtWidgets.QMainWindow):
    def __init__(self):
        super(Window, self).__init__()
        uic.loadUi("./score_tracker.ui", self)

        # Populate tables
        LICENSE_PLATE_FILE = '/../../enph353_gazebo/scripts/plates.csv'
        SCRIPT_FILE_PATH = os.path.dirname(os.path.realpath(__file__))
        with open(SCRIPT_FILE_PATH + LICENSE_PLATE_FILE, "r") as plate_file:
            platereader = csv.reader(plate_file)
            i=0
            for row in platereader:
                if i < NUM_LOCATIONS:
                    self.license_scores_QTW.item(i, 1).setText(row[0])
                else:
                    break
                i += 1

        # Connect widgets
        self.penalty_vehicle_QPB.clicked.connect(self.penalty_vehicle)
        self.penalty_pedestrian_QPB.clicked.connect(self.penalty_pedestrian)
        self.penalty_track_QPB.clicked.connect(self.penalty_track)


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
            licenseTotal += self.license_scores_QTW.item(i, 3)

        self.license_total_value_QL.setText(str(licenseTotal))

        penaltyTotal     = int(self.penalties_total_value_QL.text())
        self.total_scorea_value_QL.setText(str(penaltyTotal + licenseTotal))


    def update_penalty_total(self):
        penaltyVehicle    = int(self.penalties_scores_QTW.item(0, 3).text())
        penaltyPedestrian = int(self.penalties_scores_QTW.item(1, 3).text())
        penaltyTrack      = int(self.penalties_scores_QTW.item(2, 3).text())

        penaltyTotal = penaltyVehicle + penaltyPedestrian + penaltyTrack
        self.penalties_total_value_QL.setText(str(penaltyTotal))

        licenseTotal = int(self.license_total_value_QL.text())
        self.total_score_value_QL.setText(str(penaltyTotal + licenseTotal))


    def log_msg(self, message):
        self.comms_log_QTE.append(message)


    def licensePlate_callback(self, data):
        # TODO: Parse data
        teamID, teamPswd, plateLocation, plateID = data.split(',')
        self.comms_log_QTE.append("Callback called: {}".format(data.data))

if __name__ == "__main__":
    # data = rospy.wait_for_message('/R1/cmd_vel', Twist)
    # monitor which topics get subscribed by the user
    # emphasize the only topics teams are allowed to subscribe to

    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()

    rospy.init_node('my_listener')
    rospy.Subscriber("license_plate", String, window.licensePlate_callback)
    # teamname,teampasswd,location,platid

    sys.exit(app.exec_())