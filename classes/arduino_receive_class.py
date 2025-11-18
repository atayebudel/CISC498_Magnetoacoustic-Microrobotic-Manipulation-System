import threading
import time
from typing import List
from pySerialTransfer import pySerialTransfer as txfer


class ArduinoReceiver:
    """
    Handles connection and data reception from the 'Receiver' Arduino.

    Expects newline-terminated CSV with 6 current values:
    Coil1,Coil2,Coil3,Coil4,Coil5,Coil6\n

    Returns the last valid list of 6 floats (amps). If no data/parse error,
    returns previous values (or zeros initially).
    """
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._lock = threading.Lock()
        self._last_currents: List[float] = [0.0] * 6
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

    def receive(self):
        """
        Read one line and parse 6 comma-separated numbers.
        Returns last valid reading (list of 6 floats).
        """
        if not self.connected:
            return self._last_currents
        try:
            with self._lock:
                if self._link.available():
                    idx = 0
                    vals = []
                    for _ in range(6):
                        v = self._link.rx_obj(obj_type='f', start_pos=idx)
                        # float size = 4 bytes
                        idx += 4
                        vals.append(float(v))
                    if len(vals) == 6:
                        self._last_currents = vals
                # ignore status checks; keep last value if no packet
        except Exception:
            pass
        return self._last_currents

    def close(self):
        with self._lock:
            if getattr(self, "_link", None) is not None:
                try:
                    self._link.close()
                finally:
                    self._link = None
