#!usr/bin/python
# -----------------------------------
# A PID controller port to python of the Arduino PID Library by Brett Beauregard
#
# Eric Sanchez 7-16-18
# -----------------------------------

from __future__ import division
import time

AUTOMATIC  = 1
MANUAL = 0
DIRECT = 0
REVERSE = 1
P_ON_M = 0
P_ON_E = 1

class PID():

    def __init__(self, Kp, Ki, Kd, POn, ControllerDirection, sampleTime = None):

        self.inAuto = False

        self.SetOutputLimits(0, 255)    # Default limit correspoins to arduino limits

        if sampleTime is None:
            self.SampleTime = 0.100         # s - Default controller sample time is 0.1 seconds

        else:
            self.SampleTime = sampleTime

        self.SetControllerDirection(ControllerDirection)
        self.SetTunings(float(Kp), float(Ki), float(Kd), POn)

        self.lastTime = time.time() - self.SampleTime

        self.outputSum = 0.0
        self.lastInput = 0.0
        self.setpoint = 0.0
        self.output = 0.0


    # # * Compute() **********************************************************************
    # *     This, as they say, is where the magic happens.  this function should be called
    # *   every time "void loop()" executes.  the function will decide for itself whether a new
    # *   pid Output needs to be computed.  returns true when the output is computed,
    # *   false when nothing has been done.
    # **********************************************************************************
    def Compute(self, feedback):
        feedback = float(feedback)

        if self.inAuto == False:
            return False

        now = time.time()
        timeChange = now - self.lastTime

        if timeChange >= self.SampleTime:
            # Compute all the working error variables
            inpt = float(feedback)
            error = self.setpoint - inpt
            dInput = inpt - self.lastInput
            self.outputSum += self.ki * error

            # Add Proportional on Measurement, if P_ON_M is specified
            if self.pOnE == False:
                self.outputSum -= self.kp * dInput

            if self.outputSum > self.outMax:
                self.outputSum = self.outMax
            elif self.outputSum < self.outMin:
                self.outputSum = self.outMin

            # Add Proportional on Error, if P_ON_E is specified
            if self.pOnE:
                output_ = self.kp * error
            else:
                output_ = 0.0

            # Compute Rest of PID Output
            output_ += self.outputSum - self.kd * dInput

            if output_ > self.outMax:
                output_ = self.outMax
            elif output_ < self.outMin:
                output_ = self.outMin

            self.output = output_

            # Remember some vars for next iteration
            self.lastInput = inpt
            self.lastTime = now

            return True

        else:
            return False

    # * SetTunings(...)*************************************************************
    # * This function allows the controller's dynamic performance to be adjusted.
    # * it's called automatically from the constructor, but tunings can also
    # * be adjusted on the fly during normal operation
    # ******************************************************************************/

    def SetTunings(self, Kp, Ki, Kd, POn):
        if (Kp < 0) or (Ki < 0) or (Kd < 0):
            return

        self.pOn = POn
        self.pOnE = (POn == P_ON_E)

        self.dispKp = Kp
        self.dispKi = Ki
        self.dispKd = Kd

        SampleTimeInSec = self.SampleTime

        self.kp = Kp
        self.ki = Ki * SampleTimeInSec
        self.kd = Kd / SampleTimeInSec

        if (self.controllerDirection == REVERSE):
            self.kp = -self.kp
            self.ki = -self.ki
            self.kd = -self.kd

    # * SetTunings(...)*************************************************************
    # * Set Tunings using the last-rembered POn setting
    # ******************************************************************************
    # def SetTunings(self, Kp, Ki, Kd):
    #     SetTunings(Kp, Ki, Kd, self.pOn)

    # * SetSampleTime(...) *********************************************************
    # * sets the period, in Milliseconds, at which the calculation is performed
    # ******************************************************************************
    def SetSampleTime(self, NewSampleTime):
        NewSampleTime = float(NewSampleTime)

        if NewSampleTime > 0.0:
            ratio = NewSampleTime / self.SampleTime

            self.ki *= ratio
            self.kd /= ratio
            self.SampleTime = NewSampleTime

    # * SetOutputLimits(...)****************************************************
    # *     This function will be used far more often than SetInputLimits.  while
    # *  the input to the controller will generally be in the 0-1023 range (which is
    # *  the default already,)  the output will be a little different.  maybe they'll
    # *  be doing a time window and will need 0-8000 or something.  or maybe they'll
    # *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
    # *  here.
    # **************************************************************************
    def SetOutputLimits(self, Min,Max):
        if Min > Max:
            return

        self.outMin = Min
        self.outMax = Max

        if self.inAuto:
            if self.output > self.outMax:
                self.output = self.outMax
            elif self.output < self.outMin:
                self.output = self.outMin

            if self.outputSum > self.outMax:
                self.outputSum = self.outMax
            elif self.outputSum < self.outMin:
                self.outputSum = self.outMin

    # * SetMode(...)****************************************************************
    # * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
    # * when the transition from manual to auto occurs, the controller is
    # * automatically initialized
    # ******************************************************************************
    def SetMode(self, Mode):
        newAuto = (Mode == AUTOMATIC)

        if newAuto and (self.inAuto == False):
            self.Initialize()

        self.inAuto = newAuto

    # * Initialize()****************************************************************
    # *  does all the things that need to happen to ensure a bumpless transfer
    # *  from manual to automatic mode.
    # ******************************************************************************
    def Initialize(self):
        self.outputSum = self.output

        if self.outputSum > self.outMax:
            self.outputSum = self.outMax
        elif self.outputSum < self.outMin:
            self.outputSum = self.outMin

    # * SetControllerDirection(...)*************************************************
    # * The PID will either be connected to a DIRECT acting process (+Output leads
    # * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
    # * know which one, because otherwise we may increase the output when we should
    # * be decreasing.  This is called from the constructor.
    # ******************************************************************************
    def SetControllerDirection(self, Direction):
        if self.inAuto and (Direction != controllerDirection):
            self.kp = -self.kp
            self.ki = -self.ki
            self.kd = -self.kd
        self.controllerDirection = Direction

    def SetSetpoint(self, newSetpoint):
        self.setpoint = newSetpoint

    # * Status Funcions*************************************************************
    # * Just because you set the Kp=-1 doesn't mean it actually happened.  these
    # * functions query the internal state of the PID.  they're here for display
    # * purposes.  this are the functions the PID Front-end uses for example
    # ******************************************************************************
    def GetKp(self):
        return self.dispKp
    def GetKi(self):
        return self.dispKi
    def GetKd(self):
        return self.dispKd
    def GetMode():
        return (MANUAL, AUTOMATIC)[self.inAuto == True]
    def GetDirection():
        return self.ControllerDirection
