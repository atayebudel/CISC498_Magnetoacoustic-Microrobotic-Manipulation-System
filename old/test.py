import sys
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import random  # for demo only

class CurrentPlotter(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("5-Channel Current Plotter")
        self.resize(800, 500)

        # Create the plotting widget
        self.plot_widget = pg.PlotWidget()
        self.setCentralWidget(self.plot_widget)

        # Configure plot
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.addLegend()
        self.plot_widget.setLabel('left', 'Current (A)')
        self.plot_widget.setLabel('bottom', 'Time (samples)')

        # Buffers for plotting (store last N samples)
        self.max_points = 500  # history length like Arduino serial plotter
        self.data_buffers = [[0] * self.max_points for _ in range(5)]
        self.curves = []

        # Create 5 plot lines with different colors
        colors = ['r', 'g', 'b', 'y', 'c']
        for i in range(5):
            curve = self.plot_widget.plot(self.data_buffers[i], pen=pg.mkPen(colors[i], width=2),
                                          name=f"Sensor {i+1}")
            self.curves.append(curve)

        # Timer to update plot
        self.timer = QtCore.QTimer()
        self.timer.setInterval(40)  # ~25 FPS
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    # Replace this function with your serial or socket data input
    def get_new_data(self):
        # EXAMPLE: generate 6 fake current values
        currents = [random.uniform(0, 10) for _ in range(6)]
        return currents

    def update_plot(self):
        currents = self.get_new_data()

        # Update buffers
        for i in range(5):
            self.data_buffers[i].append(currents[i])
            self.data_buffers[i] = self.data_buffers[i][-self.max_points:]  # keep last N points

            self.curves[i].setData(self.data_buffers[i])

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = CurrentPlotter()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
