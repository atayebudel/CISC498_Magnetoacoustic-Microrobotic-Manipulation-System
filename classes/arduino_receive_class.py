import serial
import threading
import time
from typing import List


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
        self._ser = None
        self._lock = threading.Lock()
        self._last_currents: List[float] = [0.0] * 6
        self._open()

    def _open(self):
        self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        # allow Arduino time to reset
        time.sleep(2.0)

    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def receive(self):
        """
        Read one line and parse 6 comma-separated numbers.
        Returns last valid reading (list of 6 floats).
        """
        if not self.connected:
            return self._last_currents
        try:
            with self._lock:
                raw = self._ser.readline()
            if not raw:
                return self._last_currents
            line = raw.decode("utf-8", errors="ignore").strip()
            parts = [p.strip() for p in line.split(",") if p.strip() != ""]
            if len(parts) >= 6:
                vals = []
                for i in range(6):
                    vals.append(float(parts[i]))
                self._last_currents = vals[:6]
        except Exception:
            # swallow parse/IO errors and keep last good value
            pass
        return self._last_currents

    def close(self):
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                finally:
                    self._ser = None
