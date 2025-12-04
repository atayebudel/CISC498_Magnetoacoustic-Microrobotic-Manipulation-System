from classes.arduino.arduino_send_class import ArduinoSender
from classes.arduino.arduino_receive_class import ArduinoReceiver

class ArduinoHandler:
    """
    Middleman between GUI and Arduinos).
    GUI should call this instead of directly touching Arduino subclasses.
    """
    def __init__(self, ui):
        self.ui = ui
        
        # hardware
        self._arduino_sender = None
        self._arduino_receiver = None



    # Arduino Sender
    def set_sender_port(self, port: str):
        self.close_sender()
        if port:
            try:
                self._arduino_sender = ArduinoSender(port=port)
                print(f"Connected to Sender Arduino on port {port}")
            except Exception:
                self._arduino_sender = None
                print(f"Could not connect to Sender Arduino on port {port}")
                return False
        return True

    # Arduino Receiver
    def set_receiver_port(self, port: str):
        self.close_receiver()
        if port:
            try:
                self._arduino_receiver = ArduinoReceiver(port=port)
                print(f"Connected to Receiver Arduino on port {port}")
            except Exception:
                self._arduino_receiver = None
                print(f"Could not connect to Receiver Arduino on port {port}")
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
                return self._arduino_receiver.read()
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
