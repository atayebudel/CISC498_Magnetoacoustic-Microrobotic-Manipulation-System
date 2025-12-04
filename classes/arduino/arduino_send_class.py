import time
from pySerialTransfer import pySerialTransfer as txfer

class ArduinoSender:
    """
    Simple class to send actuation commands to the Arduino.
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._link = None
        self._open()

    def _open(self):
        try:
            self._link = txfer.SerialTransfer(self.port, baud=self.baudrate)
            self._link.open()     
            time.sleep(2.0)  # give Arduino time to reset

        except Exception as e:
            print(f"Could not connect to Arduino: {e}")
            self._link = None



    def send(self, Bx, By, Bz, alpha, gamma, freq, psi, gradient, equal_field, acoustic):
        """
        Send actuation command packet (binary encoded).
        All arguments should be float or int.
        """
        if self._link is not None:
          
            try:
                idx = 0
                idx = self._link.tx_obj(float(Bx), start_pos=idx)
                idx = self._link.tx_obj(float(By), start_pos=idx)
                idx = self._link.tx_obj(float(Bz), start_pos=idx)
                idx = self._link.tx_obj(float(alpha), start_pos=idx)
                idx = self._link.tx_obj(float(gamma), start_pos=idx)
                idx = self._link.tx_obj(float(freq), start_pos=idx)
                idx = self._link.tx_obj(float(psi), start_pos=idx)
                idx = self._link.tx_obj(float(gradient), start_pos=idx)
                idx = self._link.tx_obj(float(equal_field), start_pos=idx)
                idx = self._link.tx_obj(float(acoustic), start_pos=idx)

                self._link.send(idx)
            except Exception as e:
                print(f"Error sending data: {e}")

    def close(self):
        """Close the serial connection."""
        if self._link is not None:
            try:
                self._link.close()
            except Exception as e:
                print(f"Error closing connection: {e}")
            finally:
                self._link = None


# Example usage:
if __name__ == "__main__":
    arduino = ArduinoSender(port="/dev/cu.usbmodem1201", baudrate=115200)
    time.sleep(2)
    print("open arduino srial monitor so you can see data being received")
    for i in range(100):
        B = i/100
        #arduino.send(B, B, B, B, B, B, B, B, B, B)
        arduino.send(1, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        print("Command sent")
        time.sleep(0.05)
    time.sleep(2)
    arduino.send(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    print("Zero command sent")
    arduino.close()
 