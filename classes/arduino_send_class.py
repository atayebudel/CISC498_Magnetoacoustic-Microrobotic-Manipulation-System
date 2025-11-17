import serial
import threading
import time


class ArduinoSender:
    """
    Handles connection and command transmission to the 'Sender' Arduino.

    Sends actuation commands as a simple CSV line:
    Bx,By,Bz,alpha,gamma,freq,psi,gradient,equal_field,acoustic\n
    """
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser = None
        self._lock = threading.Lock()
        self._open()

    def _open(self):
        self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        # small delay for Arduino reset
        time.sleep(2.0)

    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def send(self, Bx, By, Bz, alpha, gamma, freq, psi, gradient, equal_field, acoustic):
        """
        Send actuation command packet (CSV encoded).
        Values are floats/ints; Arduino must parse accordingly.
        """
        if not self.connected:
            return
        line = f"{Bx:.6f},{By:.6f},{Bz:.6f},{alpha:.6f},{gamma:.6f},{freq:.6f},{psi:.6f},{int(gradient)},{int(equal_field)},{float(acoustic):.2f}\n"
        data = line.encode("utf-8", errors="ignore")
        with self._lock:
            self._ser.write(data)
            # optional flush
            self._ser.flush()

    def close(self):
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                finally:
                    self._ser = None
