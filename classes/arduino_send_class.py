import threading
import time
from pySerialTransfer import pySerialTransfer as txfer


class ArduinoSender:
    """
    Handles connection and command transmission to the 'Sender' Arduino.

    Sends actuation commands as a binary packet.
    """
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._link = None
        self._lock = threading.Lock()
        self._open()

    def _open(self):
        try:
            self._link = txfer.SerialTransfer(self.port, baudrate=self.baudrate)
            time.sleep(2.0)
        except Exception:
            self._link = None

    @property
    def connected(self) -> bool:
        return getattr(self, "_link", None) is not None

    def send(self, Bx, By, Bz, alpha, gamma, freq, psi, gradient, equal_field, acoustic):
        """
        Send actuation command packet (binary encoded).
        Values are floats/ints; Arduino must parse accordingly.
        """
        if not self.connected:
            return
        try:
            idx = 0
            idx = self._link.tx_obj(float(Bx), start_pos=idx)
            idx = self._link.tx_obj(float(By), start_pos=idx)
            idx = self._link.tx_obj(float(Bz), start_pos=idx)
            idx = self._link.tx_obj(float(alpha), start_pos=idx)
            idx = self._link.tx_obj(float(gamma), start_pos=idx)
            idx = self._link.tx_obj(float(freq), start_pos=idx)
            idx = self._link.tx_obj(float(psi), start_pos=idx)
            idx = self._link.tx_obj(int(gradient), start_pos=idx)
            idx = self._link.tx_obj(int(equal_field), start_pos=idx)
            idx = self._link.tx_obj(float(acoustic), start_pos=idx)
            with self._lock:
                self._link.send(idx)
        except Exception:
            pass

    def close(self):
        with self._lock:
            if getattr(self, "_link", None) is not None:
                try:
                    self._link.close()
                finally:
                    self._link = None
