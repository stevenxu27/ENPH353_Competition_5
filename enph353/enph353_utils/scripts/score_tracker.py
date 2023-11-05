#!/usr/bin/env python3
from datetime import datetime
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import (QPixmap)
from PyQt5.QtCore import (Qt, QTimer, pyqtSignal)
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from python_qt_binding import loadUi

import csv
import os
import rospy
import sys

NUM_LOCATIONS = 8

class Window(QtWidgets.QMainWindow):
    message_received_signal = pyqtSignal(str)

    def __init__(self):
        super(Window, self).__init__()

        # Register the UI elements in Python
        loadUi("./score_tracker.ui", self)

        # Add logo
        pixmap = QPixmap('FIZZ_CLUE.svg')
        self.label_QL.setPixmap(pixmap)

        # Populate log file name
        now = datetime.now()
        date_time = now.strftime("%Y%m%d_%H%M%S")
        self.log_file_path = (self.team_ID_value_QL.text() + "_" + 
                              date_time + '.txt')
        self.log_file_value_QL.setText(self.log_file_path)

        # Set score table contents
        # Adjust column widths
        self.license_scores_QTW.setColumnWidth(0, 12)
        self.license_scores_QTW.setColumnWidth(1, 80)
        self.license_scores_QTW.setColumnWidth(2, 130)
        self.license_scores_QTW.setColumnWidth(3, 130)
        self.license_scores_QTW.setColumnWidth(4, 40)

        # Populate table contents
        # @sa plate_generator.py: this is where the plates.csv is generated
        LICENSE_PLATE_FILE = '/../../enph353_gazebo/scripts/plates.csv'
        SCRIPT_FILE_PATH = os.path.dirname(os.path.realpath(__file__))
        with open(SCRIPT_FILE_PATH + LICENSE_PLATE_FILE, "r") as plate_file:
            platereader = csv.reader(plate_file)
            i=0
            for row in platereader:
                if i < NUM_LOCATIONS:
                    self.license_scores_QTW.item(i, 2).setText(row[1])
                    self.log_msg("Clue {}: {}".format(row[0], row[1]))
                else:
                    break
                i += 1

        # Register timer 
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.SLOT_timer_update)
        self.elapsed_time_s = 0

        # Initialize other variables:
        self.bonus_points = 0

        self.first_cmd_vel = True

        # Connect widgets

        # Table values changed:
        self.license_scores_QTW.itemChanged.connect(self.SLOT_license_scores_changed)
        self.penalties_scores_QTW.itemChanged.connect(self.SLOT_penalties_changed)

        # Penalties deducted:
        self.penalty_vehicle_QPB.clicked.connect(self.SLOT_penalty_collision)
        self.penalty_pedestrian_QPB.clicked.connect(self.SLOT_penalty_respawn)
        self.penalty_track_QPB.clicked.connect(self.SLOT_penalty_track)

        self.lap_completed_QPB.clicked.connect(self.SLOT_bonus_completed)
        self.manual_control_QPB.clicked.connect(self.SLOT_manual_control)

        self.message_received_signal.connect(self.SLOT_message_received)

        # Set-up ROS subscribers
        self.sub_score_tracker = rospy.Subscriber("score_tracker", String, 
                                                  self.score_tracker_callback)
        self.sub_cmd_vel = rospy.Subscriber("/R1/cmd_vel", Twist,
                                            self.cmd_vel_callback)
        
        # Register ROS node
        rospy.init_node('competition_listener')

    def cmd_vel_callback(self, data):
        '''
        Used to log that the car has started moving.
        '''
        if self.first_cmd_vel:
            self.log_msg("First command velocity received.")
            self.first_cmd_vel = False

    def score_tracker_callback(self, data):
        '''
        Use the callback to emit a signal. This is how we translate from ROS to
        Qt events. (ROS event -> subscriber callback -> Qt Signal -> Qt Slot)
        '''
        self.message_received_signal.emit(str(data.data))


    def log_msg(self, message):
        now = datetime.now()
        date_time = now.strftime("%H:%M:%S.%f")[:-3]
        log_output = "<font color='blue'>{}</font>: {}".format(date_time, message)
        self.comms_log_QTE.append(log_output)
        # self.comms_log_QTE.insertHtml(log_output)

        log_file_content = self.comms_log_QTE.toPlainText()

        with open(self.log_file_path, "w") as html_file:
            html_file.write(log_file_content)


    def SLOT_bonus_completed(self):
        if self.bonus_points == 5:
            self.log_msg("Bonus completed already awarded points.")
            return
        self.log_msg("Bonus completed: +5 points")
        self.bonus_points = 5
        self.update_license_total()


    def SLOT_license_scores_changed(self):
        self.update_license_total()


    def SLOT_manual_control(self):
        if (self.manual_control_QPB.isChecked()):
            self.log_msg("Manual control enabled (0.5x points).")
        else:
            self.log_msg("Manual control disabled (1x points).")


    def SLOT_message_received(self, license_string):
        '''
        Processes the reported data
        '''
        self.log_msg("Message received: {}".format(license_string))

        teamID, teamPswd, reportedLocation, plateTxt = str(license_string).split(',')

        # Check out of bounds plate location
        if int(reportedLocation) < -1 or int(reportedLocation) > 8:
            self.log_msg("Invalid plate location: {}".format(reportedLocation))
            return
        
        # Use to start the timer and register the team name (not for points)
        if reportedLocation == '0':
            # Update team ID and log file name:
            if teamID !=  self.team_ID_value_QL.text():
                now = datetime.now()
                date_time = now.strftime("%Y%m%d_%H%M%S")
                self.log_file_path = teamID + "_" + date_time + '.txt'
                self.log_file_value_QL.setText(self.log_file_path)

            self.team_ID_value_QL.setText(teamID)

            self.start_timer()
            return

        # Use to stop the timer
        if reportedLocation == '-1':
            self.stop_timer()
            return

        if not reportedLocation.isdigit():
            self.log_msg("Plate location is not a number.")
            return

        reportedLocation = int(reportedLocation)

        # Update scoring table with current prediction (column 3 - 0 based index)
        self.license_scores_QTW.blockSignals(True)
        self.license_scores_QTW.item(reportedLocation-1, 3).setText(plateTxt)

        # Read the ground truth for the current prediction (column 3 - 0 based index)
        gndTruth = str(self.license_scores_QTW.item(reportedLocation-1, 2).text())
        self.license_scores_QTW.blockSignals(False)

        # Manual control results in half the points per guess being awarded:
        manual_control_factor = 1
        if self.manual_control_QPB.isChecked():
            manual_control_factor = 0.5

        # Check submitted license plate ID and location against gnd truth:
        if gndTruth == plateTxt:
            # award 8 points for the last 2 plates and 6 points for the rest
            points_awarded = int(6 * manual_control_factor)
            if reportedLocation > 6:
                points_awarded = int(8 * manual_control_factor)
        else:
            # if incorrect prediction deduct the points awarded (set them to 0)
            points_awarded = 0
        
        # Updated scoring table with number of points awarded (column 4 - 0 based index)
        self.license_scores_QTW.item(reportedLocation-1, 4).setText(str(points_awarded))
        self.log_msg("Awarded: {} pts".format(points_awarded))

        self.update_story_line()


    def SLOT_penalties_changed(self):
        self.update_penalty_total()


    def SLOT_penalty_collision(self):
        table_row = 0

        # update number of events (this will trigger the update_penalty_total)
        numEvents       = int(self.penalties_scores_QTW.item(table_row, 1).text()) + 1
        self.penalties_scores_QTW.item(table_row, 1).setText(str(numEvents))

        penaltyPerEvent = int(self.penalties_scores_QTW.item(table_row, 2).text())
        self.log_msg("Penalty: collision: {} pts".format(penaltyPerEvent))


    def SLOT_penalty_respawn(self):
        table_row = 1

        # update number of events (this will trigger the update_penalty_total)
        numEvents       = int(self.penalties_scores_QTW.item(table_row, 1).text()) + 1
        self.penalties_scores_QTW.item(table_row, 1).setText(str(numEvents))
        
        penaltyPerEvent = int(self.penalties_scores_QTW.item(table_row, 2).text())
        self.log_msg("Penalty: respawn: {} pts".format(penaltyPerEvent))


    def SLOT_penalty_track(self):
        table_row = 2

        # update number of events (this will trigger the update_penalty_total)
        numEvents       = int(self.penalties_scores_QTW.item(table_row, 1).text()) + 1
        self.penalties_scores_QTW.item(table_row, 1).setText(str(numEvents))

        penaltyPerEvent = int(self.penalties_scores_QTW.item(table_row, 2).text())
        self.log_msg("Penalty: off road: {} pts".format(penaltyPerEvent))


    def SLOT_timer_update(self):
        ROUND_DURATION_s = 240
        self.elapsed_time_s += 1
        self.sim_current_time_s = rospy.get_time()
        sim_time_s = self.sim_current_time_s - self.sim_start_time_s
        self.elapsed_time_value_QL.setText(
            "{:03d} sec".format(int(sim_time_s)))
        if (sim_time_s > ROUND_DURATION_s):
            self.log_msg("Out of time: {}sec sim time (real time: {}sec).".
                format(sim_time_s, self.elapsed_time_s))
            self.timer.stop()


    def start_timer(self):
        self.elapsed_time_s = 0
        self.sim_start_time_s = rospy.get_time()
        self.elapsed_time_value_QL.setText(
            "{:03d} sec".format(self.elapsed_time_s))
        self.timer.start(1000)
        self.log_msg("Timer started.")


    def stop_timer(self):
        self.sim_current_time_s = rospy.get_time()
        sim_time_s = self.sim_current_time_s - self.sim_start_time_s
        self.log_msg("Timer stopped: {}sec sim time (real time: {}sec).".
                format(sim_time_s, self.elapsed_time_s))
        self.timer.stop()


    def update_license_total(self):
        licenseTotal = 0
        for i in range(NUM_LOCATIONS):
            licenseTotal += int(self.license_scores_QTW.item(i, 4).text())

        self.license_total_value_QL.setText(str(licenseTotal))

        penaltyTotal = int(self.penalties_total_value_QL.text())
        teamTotal = penaltyTotal + licenseTotal + self.bonus_points
        self.total_score_value_QL.setText(str(teamTotal))
        self.log_msg("Team total: {} pts".format(str(teamTotal)))


    def update_penalty_total(self):
        self.penalties_scores_QTW.blockSignals(True)

        #update collision penalties total:
        numEvents         = int(self.penalties_scores_QTW.item(0, 1).text())
        penaltyPerEvent   = int(self.penalties_scores_QTW.item(0, 2).text())
        penaltyCollision  = numEvents * penaltyPerEvent
        self.penalties_scores_QTW.item(0, 3).setText(str(penaltyCollision))

        #update respawn penalties total:
        numEvents         = int(self.penalties_scores_QTW.item(1, 1).text())
        penaltyPerEvent   = int(self.penalties_scores_QTW.item(1, 2).text())
        penaltyRespawn    = numEvents * penaltyPerEvent
        self.penalties_scores_QTW.item(1, 3).setText(str(penaltyRespawn))

        #update track penalties total
        numEvents         = int(self.penalties_scores_QTW.item(2, 1).text())
        penaltyPerEvent   = int(self.penalties_scores_QTW.item(2, 2).text())
        penaltyOffRoad    = numEvents * penaltyPerEvent
        self.penalties_scores_QTW.item(2, 3).setText(str(penaltyOffRoad))

        penaltyTotal = penaltyCollision + penaltyRespawn + penaltyOffRoad
        self.penalties_total_value_QL.setText(str(penaltyTotal))
        self.log_msg("Penalties total: {} pts".format(penaltyTotal))

        licenseTotal = int(self.license_total_value_QL.text())
        teamTotal = penaltyTotal + licenseTotal
        self.total_score_value_QL.setText(str(teamTotal))
        self.log_msg("Team total: {} pts".format(teamTotal))

        self.penalties_scores_QTW.blockSignals(False)


    def update_story_line(self):
        inspector = self.team_ID_value_QL.text()
        story = "Detective {} received a message about a new crime in Linear City.".format(inspector)

        self.story_line_value_QTE.append(story)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()

    sys.exit(app.exec_())