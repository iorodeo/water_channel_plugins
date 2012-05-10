"""
Test plugin 1 - simulated using timer object instead of actually controlling the robot.
"""
import time
from threading import Timer

PLUGIN_NAME = 'test plugin 1'
PLUGIN_MODE = 'any'

class SledControlPlugin(object):

    def __init__(self, robotControlObj, startupMode, logFileName, inProgressFcn, doneFcn, messageFcn):
        self.robotControlObj = robotControlObj
        self.logNameFile = logFileName
        self.inProgressFcn = inProgressFcn
        self.doneFcn = doneFcn
        self.messageFcn = messageFcn
        self.count = 0
        self.maxCount = 5
        self.timerDelay = 3.0
        self.messageFcn('{0}: initializing'.format(PLUGIN_NAME))

    def startNextOutscan(self): 
        self.inProgressFcn(True)
        msg = '{0}: running outscan {1}'.format(PLUGIN_NAME, self.count)
        self.messageFcn(msg)
        self.timer = Timer(self.timerDelay, self.timerFcn)
        self.timer.start()

    def cleanup(self):
        msg = '{0}: cleaning up'.format(PLUGIN_NAME)
        self.messageFcn(msg)

    def timerFcn(self):
        self.count+=1
        self.inProgressFcn(False)
        if self.count >= self.maxCount:
            self.doneFcn()



