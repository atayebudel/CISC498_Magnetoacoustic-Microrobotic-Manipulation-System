import time
from pySerialTransfer import pySerialTransfer as txfer

class ArduinoReceiver:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self._link = None
        self._open()

    def _open(self):
        try:
            self._link = txfer.SerialTransfer(self.port)
            time.sleep(2.0)
        except Exception as e:
            print(f"Could not connect: {e}")
            self._link = None

    def read(self):
        if self._link is None:
            return None

        #self._link.flush_input_buffer() # flush old data

        if self._link.available():
            idx = 0
            currents = []

            for _ in range(6):
                val = self._link.rx_obj(float, start_pos=idx)
                idx += 4
                currents.append(val)
            
     
            return currents

        return None

    def close(self):
        if self._link is not None:
            self._link.close()
            self._link = None


# Example usage:
if __name__ == "__main__":
    arduino = ArduinoReceiver(port="/dev/cu.usbmodem1101", baudrate=115200)


    while True:
        data = arduino.read()
      
        if data:
            print("Currents:", data)
        else:
            print("No data received")
        time.sleep(0.05)
