#!/usr/bin/env python

from networktables import NetworkTables
import csv

ENABLED_FIELD = 1 << 0
AUTO_FIELD = 1 << 1
TEST_FIELD = 1 << 2
EMERGENCY_STOP_FIELD = 1 << 3
FMS_ATTACHED_FIELD = 1 << 4
DS_ATTACHED_FIELD = 1 << 5


def translate_control_word(value):
    value = int(value)
    if value & ENABLED_FIELD == 0:
        return "disabled"
    if value & AUTO_FIELD:
        return "auto"
    if value & TEST_FIELD:
        return "test"
    else:
        return "teleop"


class Logger:
    def __init__(self):
        self.data = []
        self.finalized = False
        self.mode = ""

    def valueChanged(self, key, value, isNew):
        if key == '/FMSInfo/FMSControlData':
            mode = translate_control_word(value)
            if self.mode == "auto" and mode == "disabled":
                self.finalized = True
            self.mode = mode
        if key == '/RobotTelemetry/Drivetrain' and not self.finalized:
            print(value)
            self.data.append(value)

    def run(self):
        NetworkTables.startClientTeam(4069)
        NetworkTables.addEntryListener(self.valueChanged)
        print("Connected to NT")

        while not self.finalized:
            pass
        print("Dumping data")
        dat = self.data
        # y = [x[0] for x in dat]
        # u = [x[1] for x in dat]
        # x = [x[2] for x in dat]
        # v = [x[3] for x in dat]
        lref = [x[0] for x in dat]
        lact = [x[1] for x in dat]
        rref = [x[2] for x in dat]
        ract = [x[3] for x in dat]
        with open('output.csv', 'w') as file:
            wx = csv.DictWriter(file,
                                fieldnames=['Left Reference', 'Left Velocity', 'Right Reference', 'Right Velocity'])
            wx.writeheader()
            for i in range(len(lref)):
                wx.writerow({'Left Reference': str(lref[i]), 'Left Velocity': str(lact[i]),
                             'Right Reference': str(rref[i]), 'Right Velocity': str(ract[i])})
        print("File written")


if __name__ == "__main__":
    logger = Logger()
    logger.run()
