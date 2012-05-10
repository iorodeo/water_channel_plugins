"""
Test plugin 1 - simulated using timer object instead of actually controlling
the robot.
"""
import roslib
roslib.load_manifest('water_channel_plugins')
import rospy
import time
import h5py
from threading import Timer
from utilities import run_defs

PLUGIN_NAME = 'example position trajectory plugin'
PLUGIN_MODE = 'position trajectory'

class SledControlPlugin(object):

    # ---------------------------------------------------------------------------------------
    # Begin required functions 
    # ---------------------------------------------------------------------------------------

    def __init__(self, robotControlObj, startupMode, logFileName, inProgressFcn, doneFcn, messageFcn):
        """
        Initialization function
        """
        print 'runing __init__'

        # Check that startup mode is correct
        if not startupMode == 'position trajectory':
            raise ValueError, 'incorrect startup mode - must be position trajectory'

        # Check for log file
        if logFileName is None:
            raise ValueError, 'no log file set, log file required for plugin to function'
        
        self.robotControlObj = robotControlObj
        self.logFileName = logFileName
        self.inProgressFcn = inProgressFcn
        self.doneFcn = doneFcn
        self.messageFcn = messageFcn
        self.messageFcn('{0}: initializing'.format(PLUGIN_NAME))
        self.nextOutscanFcn = self.moveToStartOutscan

        self.count = 0
        self.maxCount = 4
        self.trialDelay = 3.0

        self.startPosition = 7.0
        self.positioningVel = 0.2
        self.positioningAcc = 0.05

        self.runEndPosition = 5.0
        self.runVel = 0.3
        self.runAcc = 0.075


    def startNextOutscan(self): 
        print 'running startNextOutscan'
        self.inProgressFcn(True)
        self.nextOutscanFcn()

    def cleanup(self):
        print 'running cleanup'
        msg = 'plugin: cleaning up'
        self.messageFcn(msg)

    # ------------------------------------------------------------------------------------
    # End required functions 
    # -------------------------------------------------------------------------------------

    def moveToStartOutscan(self):
        """
        Outscan function for moving to the start position
        """
        print 'running moveToStartOutscan'
        msg = 'plugin: move to start, count = {0}'.format(self.count)
        self.messageFcn(msg)
        self.nextOutscanFcn = self.trialDelayOutscan
        # Create ramp to starting position
        setptValues = run_defs.get_ramp( 
                self.robotControlObj.position,
                self.startPosition,
                self.positioningVel,
                self.positioningAcc,
                self.robotControlObj.dt,
                )
        # Start ramp outscan
        self.robotControlObj.startSetptOutscan( 
                setptValues,
                feedback_cb = self.outscanProgress_Callback,
                done_cb = self.outscanDone_Callback,
                )

    
    def trialDelayOutscan(self):
        """
        The trial delay outscan - used to wait for the fluid to settle. 
        """
        print 'running trialDelayOutscan'
        msg = 'plugin: trial delay, count = {0}'.format(self.count)
        self.messageFcn(msg)
        self.nextOutscanFcn = self.trialOutscan
        self.timer = Timer(self.trialDelay, self.timerDone_Callback)
        self.timer.start()

    def trialOutscan(self):
        """
        The trial outscan - this is the experimental run.
        """
        print 'running trialOutscan'
        msg = 'plugin: running trial, count = {0}'.format(self.count)
        self.messageFcn(msg)
        self.nextOutscanFcn = self.moveToStartOutscan

        # Load results of last trial - do something with it
        if self.count >= 1: 
            # Try to open log file
            try:
                f = h5py.File(self.logFileName)
            except h5py.h5e.LowLevelIOError, e:
                msg = 'unable to open log file: %s'%(str(e),)
                msgBoxTitle = 'Plugin log file error'
                QtGui.QMessageBox.critical(self,msgBoxTitle,msg)
                self.doneFcn()
                return

            # Sort trials and get the latest
            trial_list = [x for x in list(f) if 'trial' in x]
            trial_list.sort()
            data  = f[trial_list[-1]]['data']
            t, force = data['time'], data['force']

        # Design outscan set point array
        setptValues = run_defs.get_ramp( 
                self.robotControlObj.position,
                self.runEndPosition,
                self.runVel,
                self.runAcc,
                self.robotControlObj.dt,
                )

        # Start logger and outscan
        self.robotControlObj.enableLogger()
        self.robotControlObj.startSetptOutscan( 
                setptValues,
                feedback_cb = self.outscanProgress_Callback,
                done_cb = self.outscanDone_Callback,
                )

    def outscanProgress_Callback(self,data):
        """
        Callback for outscan progress  can access:  data.percent_complete and 
        data.secs_to_completion
        """
        #print 'running outscanProgres_Callback'
        pass

    def outscanDone_Callback(self,state,result):
        """
        Callback for when outscans complete
        """
        print 'running outscanDone_Callback'
        self.robotControlObj.disableControllerMode()
        self.robotControlObj.disableLogger()
        self.inProgressFcn(False)
        if state == 'succeeded':
            msg = 'plugin: outscan complete, count = {0}'.format(self.count)
            self.messageFcn(msg)
        else:
            msg = 'plugin: outscan aborted, count = {0}'.format(self.count)
            self.messageFcn(msg)
            self.doneFcn()

        # Increment run counter and check to see if we are finished
        if self.nextOutscanFcn == self.moveToStartOutscan:
            self.count+=1
            if self.count >= self.maxCount:
                self.doneFcn()

    def timerDone_Callback(self):
        """
        Callback for when the pre-trial delay is complete. Indicate the outscan (in this
        case the delay) is no longer in progress
        """
        print 'running timerDone_Callback'
        self.inProgressFcn(False)


def trial_cmp(x,y):
    """
    Comparison function for sorting trials
    """
    num_x = int(x.split('_')[-1])
    num_y = int(y.split('_')[-1])
    if num_x > num_y:
        return 1
    elif num_x < num_y:
        return -1
    else:
        return 0
