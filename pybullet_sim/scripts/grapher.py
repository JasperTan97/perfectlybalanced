import matplotlib.pyplot as plt
import numpy as np


class LivePIDGrapher:
    """
    Class to visualise error in PID
    """
    def __init__(self):
        plt.ion()
        fig, self.ax = plt.subplots()
        self.error_data = []
        self.line, = self.ax.plot(self.error_data, label="Error")
        self.ax.set_xlabel("Time step")
        self.ax.set_ylabel("Error in degrees")
        self.ax.legend()

    def update_plot(self, new_error):
        self.error_data.append(new_error/2/np.pi*360)
        self.line.set_xdata(np.arange(len(self.error_data)))
        self.line.set_ydata(self.error_data)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.001)

