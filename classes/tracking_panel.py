from classes.record_class import RecordThread
from classes.cell_class import Cell
from classes.tracker_class import VideoThread
from classes.robot_class import Robot
import os
from datetime import datetime
from classes.arduino_class import ArduinoHandler
from PyQt5 import QtCore

# Record thread
def toggle_recording(self):
        if self.cap is not None:
            if self.recorder and self.recorder.running:
                self.recorder.stop()
                self.recorder = None
                self.stop_data_record()
                self.ui.recordbutton.setText("Record Video")
                self.tbprint("Recording Stopped")
            else:
                self.frame_queue.queue.clear()
                 # set filename
                self.output_file_name = self.ui.videoNameLineEdit.text().strip()
                if not self.output_file_name:
                    self.output_file_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")

                file_path  = os.path.join(self.new_dir_path, self.output_file_name+".mp4")
                self.recorder = RecordThread(self.frame_queue, file_path, self.videofps)
                self.recorder.start()
                self.start_data_record()
                self.ui.recordbutton.setText("Stop Record")
                self.tbprint("Recording Video...")

# Cell and Robot tracking thread
def eventFilter(self, object, event):
        
        if object is self.ui.VideoFeedLabel: 
            if self.tracker is not None:
                
                if event.type() == QtCore.QEvent.MouseButtonPress:   
                    # left mouse button clicks on robot
                    if event.buttons() == QtCore.Qt.LeftButton:
                        newx, newy = self.convert_coords(event.pos())
                        #generate original bounding box
                        
                        #reset algorithm nodes
                        self.control_robot.reset()

                        if self.ui.robotmask_radio.isChecked():
                            x_1 = int(newx - self.ui.robotcroplengthbox.value()  / 2)
                            y_1 = int(newy - self.ui.robotcroplengthbox.value()  / 2)
                            w = self.ui.robotcroplengthbox.value()
                            h = self.ui.robotcroplengthbox.value()

                            robot = Robot()  # create robot instance
                            robot.add_frame(self.frame_number)
                            robot.add_time(0)
                            robot.add_position([newx,newy])
                            robot.add_velocity([0,0,0])
                            robot.add_acceleration([0,0,0])
                            robot.add_crop([x_1, y_1, w, h])
                            robot.add_area(0)
                            robot.add_blur(0)
                        
                            robot.crop_length = self.ui.robotcroplengthbox.value()
                            self.tracker.robot_list.append(robot) #this has to include tracker.robot_list because I need to add it to that class
                        
                        elif self.ui.cellmask_radio.isChecked():
                            x_1 = int(newx - self.ui.cellcroplengthbox.value()  / 2)
                            y_1 = int(newy - self.ui.cellcroplengthbox.value()  / 2)
                            w = self.ui.cellcroplengthbox.value()
                            h = self.ui.cellcroplengthbox.value()

                            cell = Cell()  # create robot instance
                            cell.add_frame(self.frame_number)
                            cell.add_time(0)
                            cell.add_position([newx,newy])
                            cell.add_velocity([0,0,0])
                            cell.add_crop([x_1, y_1, w, h])
                            cell.add_area(0)
                            cell.add_blur(0)
                           
                            cell.crop_length = self.ui.cellcroplengthbox.value()
                            
                            self.tracker.cell_list.append(cell) #this has to include tracker.robot_list because I need to add it to that class

                    
                    # right mouse button begins drawing
                    if event.buttons() == QtCore.Qt.RightButton: 
                        self.drawing = True
                        if self.ui.RRTradio.isChecked():
                            self.path_planner_status = True

                        newx, newy = self.convert_coords(event.pos())
                        if len(self.tracker.robot_list) > 0:
                            self.tracker.robot_list[-1].add_trajectory([newx, newy])
                      
                
                    #middle mouse button clears data
                    if event.buttons() == QtCore.Qt.MiddleButton: 
                        del self.tracker.robot_list[:]
                        del self.tracker.cell_list[:]
                        del self.magnetic_field_list[:]
                        del self.robots[:]
                        del self.cells[:]
                        self.apply_actions(False)
                       
                elif event.type() == QtCore.QEvent.MouseButtonRelease:
                    if event.button() == QtCore.Qt.RightButton: 
                        self.drawing = False
                        if self.ui.RRTradio.isChecked():
                            self.path_planner_status = False
                        
                        
                elif event.type() == QtCore.QEvent.MouseMove:
                    self.zoom_x, self.zoom_y = self.convert_coords(event.pos())

                    if event.buttons() == QtCore.Qt.RightButton:
                        if self.drawing == True:
                            if len(self.tracker.robot_list)>0:
                                newx, newy = self.convert_coords(event.pos())
                                
                                self.tracker.robot_list[-1].add_trajectory([newx, newy])
                            
                        
                elif event.type() ==  QtCore.QEvent.Wheel:
                    steps = event.angleDelta().y() 
                    
                    self.scrollamount += (steps and steps / abs(steps/0.5))
                    self.scrollamount = max(min(self.scrollamount,20.0),1.0)
                    self.zoomscale = self.scrollamount

        
        return super().eventFilter(object, event)

# Tracker thread
def start(self):

        if self.ui.startbutton.isChecked():
            #connect to arduino
            self.arduino = ArduinoHandler(self.tbprint, self.arduino_port)
            self.arduino.connect()
           
  

            if self.videopath is not None:

                self.frame_number = 0
                self.setFile()
                
                self.tracker = VideoThread(self)
                self.tracker.cropped_frame_signal.connect(self.update_croppedimage)
                self.tracker.actions_frame_signal.connect(self.update_actions_frame)
                self.tracker.start()
               
                
                

                self.ui.startbutton.setText("Stop")
                self.ui.VideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(0, 255, 0); ")
                self.ui.CroppedVideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(0, 255, 0); ")
        
                
        else:
            self.ui.VideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(255, 0, 0); ")
            self.ui.CroppedVideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(255, 0, 0); ")
            
            
          


            if self.tracker is not None:
                self.ui.startbutton.setText("Start")
                self.tracker.stop()
                
                
                #reset control button
                self.control_status = False
                self.ui.controlbutton.setText("Control")
                self.tbprint("Control Off")
                self.ui.controlbutton.setChecked(False)

                #reset joystick button
                self.joystick_status = False
                self.ui.joystickbutton.setText("Joystick")
                self.tbprint("Joystick Off")
                self.ui.joystickbutton.setChecked(False)

                #reset mask button
                self.tracker.mask_flag = False
                self.ui.maskbutton.setText("Mask")
                self.ui.maskbutton.setChecked(False)

                #also reset pause button
                self.ui.pausebutton.setChecked(False)
                self.ui.pausebutton.setText("Pause")

                self.ui.pausebutton.hide()
                self.ui.leftbutton.hide()
                self.ui.rightbutton.hide()
            

                #zero arduino commands
                self.apply_actions(False)
                del self.tracker.robot_list[:]
                del self.magnetic_field_list[:]

                self.ui.applyacousticbutton.setChecked(False)
                self.ui.led.setStyleSheet("\n"
                "                background-color: rgb(255, 0, 0);\n"
                "                border-style: outset;\n"
                "                border-width: 3px;\n"
                "                border-radius: 12px;\n"
                "                border-color: rgb(255, 0, 0);\n"
                "         \n"
                "                padding: 6px;")

            if self.arduino is not None:
                self.arduino.close()
