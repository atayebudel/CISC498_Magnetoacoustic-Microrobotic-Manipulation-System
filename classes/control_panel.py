from classes.algorithm_class import AlgorithmHandler
from classes.arduino_send_class import ArduinoSender
from classes.arduino_receive_class import ArduinoReceiver

class ControlPanel:
    """
    Middleman between GUI and subsystems (algorithms + Arduinos).
    GUI should call this instead of directly touching AlgorithmHandler / Arduino classes.
    """
    def __init__(self, ui):
        self.ui = ui
        # algorithms
        self.algorithm_handler = AlgorithmHandler(ui)
        self.simulator = self.algorithm_handler.simulator
        self.control_robot = self.algorithm_handler.control_robot
        self.path_planner = self.algorithm_handler.path_planner
        self.joystick_actions = self.algorithm_handler.joystick_actions
        # hardware
        self._arduino_sender = None
        self._arduino_receiver = None

    # Arduino Sender
    def set_sender_port(self, port: str):
        self.close_sender()
        if port:
            try:
                self._arduino_sender = ArduinoSender(port=port)
            except Exception:
                self._arduino_sender = None
                return False
        return True

    # Arduino Receiver
    def set_receiver_port(self, port: str):
        self.close_receiver()
        if port:
            try:
                self._arduino_receiver = ArduinoReceiver(port=port)
            except Exception:
                self._arduino_receiver = None
                return False
        return True

    def send_actions(self, Bx, By, Bz, alpha, gamma, freq, psi,
                     gradient, equal_field, acoustic):
        if self._arduino_sender is not None:
            try:
                self._arduino_sender.send(Bx, By, Bz, alpha, gamma, freq, psi,
                                          gradient, equal_field, acoustic)
            except Exception:
                pass  # swallow hardware errors

    def receive_currents(self):
        if self._arduino_receiver is not None:
            try:
                return self._arduino_receiver.receive()
            except Exception:
                return None
        return None

    def close_sender(self):
        if self._arduino_sender is not None:
            try:
                self._arduino_sender.close()
            except Exception:
                pass
            self._arduino_sender = None

    def close_receiver(self):
        if self._arduino_receiver is not None:
            try:
                self._arduino_receiver.close()
            except Exception:
                pass
            self._arduino_receiver = None

    def close(self):
        self.close_sender()
        self.close_receiver()
