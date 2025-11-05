from classes.record_class import RecordThread
from classes.cell_class import Cell
from classes.tracker_class import VideoThread
from classes.robot_class import Robot
import os
from datetime import datetime
from classes.arduino_class import ArduinoHandler
from PyQt5 import QtCore, QtWidgets

# Record thread
def toggle_recording(self):
    """Start or stop video recording with synchronized data logging"""
    if self.cap is not None:
        if self.recorder and self.recorder.running:
            # Stop recording
            self.recorder.stop()
            self.recorder = None
            self.stop_data_record()
            self.ui.recordbutton.setText("Record Video")
            self.tbprint("Recording Stopped")
        else:
            # Start recording
            self.frame_queue.queue.clear()
            
            # Generate filename from user input or timestamp
            self.output_file_name = self.ui.videoNameLineEdit.text().strip()
            if not self.output_file_name:
                self.output_file_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")

            file_path = os.path.join(self.new_dir_path, self.output_file_name + ".mp4")
            self.recorder = RecordThread(self.frame_queue, file_path, self.videofps)
            self.recorder.start()
            self.start_data_record()
            self.ui.recordbutton.setText("Stop Record")
            self.tbprint("Recording Video...")

# Cell and Robot tracking thread
def eventFilter(self, object, event):
    """Handle mouse and scroll events on the video feed
    
    Mouse controls:
        - Left click: Add robot or cell at position
        - Right click + drag: Draw trajectory path
        - Middle click: Clear all tracking data
        - Scroll wheel: Zoom in/out
    """
    if object is self.ui.VideoFeedLabel: 
        if self.tracker is not None:
            
            if event.type() == QtCore.QEvent.MouseButtonPress:   
                # Left click: Initialize robot or cell tracking
                if event.buttons() == QtCore.Qt.LeftButton:
                    newx, newy = self.convert_coords(event.pos())
                    
                    # Reset control algorithm state
                    self.control_robot.reset()

                    if self.ui.robotmask_radio.isChecked():
                        # Create new robot instance
                        x_1 = int(newx - self.ui.robotcroplengthbox.value() / 2)
                        y_1 = int(newy - self.ui.robotcroplengthbox.value() / 2)
                        w = self.ui.robotcroplengthbox.value()
                        h = self.ui.robotcroplengthbox.value()

                        robot = Robot()
                        robot.add_frame(self.frame_number)
                        robot.add_time(0)
                        robot.add_position([newx, newy])
                        robot.add_velocity([0, 0, 0])
                        robot.add_acceleration([0, 0, 0])
                        robot.add_crop([x_1, y_1, w, h])
                        robot.add_area(0)
                        robot.add_blur(0)
                        robot.crop_length = self.ui.robotcroplengthbox.value()
                        self.tracker.robot_list.append(robot)
                    
                    elif self.ui.cellmask_radio.isChecked():
                        # Create new cell instance
                        x_1 = int(newx - self.ui.cellcroplengthbox.value() / 2)
                        y_1 = int(newy - self.ui.cellcroplengthbox.value() / 2)
                        w = self.ui.cellcroplengthbox.value()
                        h = self.ui.cellcroplengthbox.value()

                        cell = Cell()
                        cell.add_frame(self.frame_number)
                        cell.add_time(0)
                        cell.add_position([newx, newy])
                        cell.add_velocity([0, 0, 0])
                        cell.add_crop([x_1, y_1, w, h])
                        cell.add_area(0)
                        cell.add_blur(0)
                        cell.crop_length = self.ui.cellcroplengthbox.value()
                        self.tracker.cell_list.append(cell)

                # Right click: Start drawing trajectory
                if event.buttons() == QtCore.Qt.RightButton: 
                    self.drawing = True
                    if self.ui.RRTradio.isChecked():
                        self.path_planner_status = True

                    newx, newy = self.convert_coords(event.pos())
                    if len(self.tracker.robot_list) > 0:
                        self.tracker.robot_list[-1].add_trajectory([newx, newy])

                # Middle click: Clear all tracked objects
                if event.buttons() == QtCore.Qt.MiddleButton: 
                    del self.tracker.robot_list[:]
                    del self.tracker.cell_list[:]
                    del self.magnetic_field_list[:]
                    del self.robots[:]
                    del self.cells[:]
                    self.apply_actions(False)
                   
            elif event.type() == QtCore.QEvent.MouseButtonRelease:
                # Stop drawing trajectory
                if event.button() == QtCore.Qt.RightButton: 
                    self.drawing = False
                    if self.ui.RRTradio.isChecked():
                        self.path_planner_status = False

            elif event.type() == QtCore.QEvent.MouseMove:
                # Update zoom center coordinates
                self.zoom_x, self.zoom_y = self.convert_coords(event.pos())

                # Continue drawing trajectory while dragging
                if event.buttons() == QtCore.Qt.RightButton:
                    if self.drawing == True:
                        if len(self.tracker.robot_list) > 0:
                            newx, newy = self.convert_coords(event.pos())
                            self.tracker.robot_list[-1].add_trajectory([newx, newy])

            elif event.type() == QtCore.QEvent.Wheel:
                # Handle zoom with scroll wheel
                steps = event.angleDelta().y() 
                self.scrollamount += (steps and steps / abs(steps / 0.5))
                self.scrollamount = max(min(self.scrollamount, 20.0), 1.0)
                self.zoomscale = self.scrollamount
    
    return QtWidgets.QMainWindow.eventFilter(self, object, event)

# Tracker thread
def start(self):
    """Start or stop video tracking and Arduino connection"""
    if self.ui.startbutton.isChecked():
        # Initialize Arduino connection
        self.arduino = ArduinoHandler(self.tbprint, self.arduino_port)
        self.arduino.connect()

        if self.videopath is not None:
            self.frame_number = 0
            self.setFile()
            
            # Start video tracking thread
            self.tracker = VideoThread(self)
            self.tracker.cropped_frame_signal.connect(self.update_croppedimage)
            self.tracker.actions_frame_signal.connect(self.update_actions_frame)
            self.tracker.start()

            # Update UI to indicate active state
            self.ui.startbutton.setText("Stop")
            self.ui.VideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(0, 255, 0); ")
            self.ui.CroppedVideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(0, 255, 0); ")
    else:
        # Update UI to indicate stopped state
        self.ui.VideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(255, 0, 0); ")
        self.ui.CroppedVideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(255, 0, 0); ")

        if self.tracker is not None:
            self.ui.startbutton.setText("Start")
            self.tracker.stop()
            
            # Reset all control modes
            self.control_status = False
            self.ui.controlbutton.setText("Control")
            self.tbprint("Control Off")
            self.ui.controlbutton.setChecked(False)

            self.joystick_status = False
            self.ui.joystickbutton.setText("Joystick")
            self.tbprint("Joystick Off")
            self.ui.joystickbutton.setChecked(False)

            self.tracker.mask_flag = False
            self.ui.maskbutton.setText("Mask")
            self.ui.maskbutton.setChecked(False)

            self.ui.pausebutton.setChecked(False)
            self.ui.pausebutton.setText("Pause")
            self.ui.pausebutton.hide()
            self.ui.leftbutton.hide()
            self.ui.rightbutton.hide()

            # Clear tracking data and zero outputs
            self.apply_actions(False)
            del self.tracker.robot_list[:]
            del self.magnetic_field_list[:]

            self.ui.applyacousticbutton.setChecked(False)
            self.ui.led.setStyleSheet("""
                background-color: rgb(255, 0, 0);
                border-style: outset;
                border-width: 3px;
                border-radius: 12px;
                border-color: rgb(255, 0, 0);
                padding: 6px;
            """)

        if self.arduino is not None:
            self.arduino.close()
