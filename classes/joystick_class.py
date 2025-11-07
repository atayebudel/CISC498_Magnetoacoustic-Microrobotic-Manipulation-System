import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
import numpy as np
from scipy import interpolate

class genJoystick: #generic
    """
    handles joystick inputs and outputs to main program
    """
    def __init__(self):
        #initlize actions actions
        self.Bx, self.By, self.Bz = 0,0,0
        self.Mx, self.My, self.Mz = 0,0,0
        self.alpha, self.gamma, self.freq, self.psi = 0,0,0,0
        self.acoustic_frequency = 0
        self.typ = 0
        self.joysticks = {}

    def is_active(self):
        return self.joysticks.len() > 0

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                joy = pygame.joystick.Joystick(event.device_index)
                self.joysticks[joy.get_instance_id()] = joy
                self.joystick = self.joysticks[joy.get_instance_id()]
                print("Controller connected")
            elif event.type == pygame.JOYDEVICEREMOVED:
                del self.joysticks[event.instance_id]
                self.joystick = 0
                print("Controller disconnected")


    def deadzone(self, value):
        """
        accepts a value [0,1] and if its less than .2 make it zero otherwise use the value. limits joystick noise
        """
        if abs(value) < .1:
            return 0
        else:
            return value

    def run(self):
        """
        main Joystick event loop the listen for commands from the joystick
        once commands are found there are converted into the proper action format and sent to the queue.
        """       
        for event in pygame.event.get():
            joystick = self.joysticks[event.get_instance_id()]
            #axis condition
            if event.type == pygame.JOYAXISMOTION:
                # Joystick movement event
                if event.axis == 1:  #LY
                    ly = -self.deadzone(event.value)
                    self.By = round(ly,3)
                    print("LY")
                
                    
                if event.axis == 0: #LX
                    lx = self.deadzone(event.value)
                    self.Bx = round(lx,3)
                    print("LX")
            

                if event.axis == 2 or event.axis == 3: #RY
                    ry = -self.deadzone(joystick.get_axis(3))
                    rx = self.deadzone(joystick.get_axis(2))
                    
                    if rx == 0 and ry == 0:
                        self.alpha = 0
                        self.gamma = 0
                        self.freq = 0

                    elif rx == 0 and ry > 0:
                        self.alpha = np.pi/2
                        self.freq = int(np.sqrt((rx)**2 + (ry)**2)*40)
                        
                    elif rx == 0 and ry < 0:
                        self.alpha = -np.pi/2
                        self.freq = int(np.sqrt((rx)**2 + (ry)**2)*40)
                    else:
                        angle = np.arctan2(ry,rx) 
                        self.alpha = round(angle,3)
                        self.freq = int(np.sqrt((rx)**2 + (ry)**2)*40)
                    

                if event.axis == 4: #LT
                    lt = round(event.value,2)
                    f = interpolate.interp1d([-1,1], [0,1])  #need to map the -1 to 1 output to 0-1
                    self.Bz = -round(float(f(lt)),3)
                    

                if event.axis == 5: #RT
                    rt = round(event.value,2)
                    f = interpolate.interp1d([-1,1], [0,1])  #need to map the -1 to 1 output to 0-1
                    self.Bz = round(float(f(rt)),3)
    
            #button condition
            elif event.type == pygame.JOYBUTTONDOWN:
                # Joystick button press event
                button = event.button
                if button == 0: #X
                    self.acoustic_frequency = 1
                    print("X")
                if button == 1: #circle
                    self.typ = 2 #spin counter clockwise
                    self.freq = 1
                    print("CIRCLE")
                if button == 2: #square
                    self.typ = 1 #spin clockwise
                    self.freq = 1
                    print("SQUARE")
                if button == 3: #triangle
                    pass
                if button == 9: #lb
                    self.Bz = -1
                if button == 10: #rb
                    self.Bz = 1
                if button == 11: #up
                    self.Mx = 1
                if button == 12: #down
                    self.My = 1
                if button == 14: #right
                    self.Mz = 1
                
            
            #zero condition
            else:
                self.typ = 0
                self.Bx = 0
                self.By = 0
                self.Bz = 0
                self.Mx = 0
                self.My = 0
                self.Mz = 0
                self.alpha = 0 
                self.gamma = 0
                self.freq = 0
                self.acoustic_frequency = 0
        

        self.actions = [self.typ, 
                        self.Bx, 
                        self.By,
                        self.Bz,
                        self.alpha,
                        self.gamma,
                        self.freq,
                        self.psi,
                        self.acoustic_frequency]

        return self.actions

