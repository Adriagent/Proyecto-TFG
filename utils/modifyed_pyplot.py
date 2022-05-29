import matplotlib.pyplot as plt
from roboticstoolbox.backends.PyPlot.RobotPlot import RobotPlot



def launch(self,fig=None, limits=None):

        self.fig = fig 

        self.fig.subplots_adjust(left=-0.01, bottom=0, top=1, right=0.99)

        # Create a 3D axes
        self.ax = self.fig.add_subplot(111, projection='3d', proj_type='ortho')
        self.ax.set_facecolor('white')

        self.ax.set_xbound(-0.5, 0.5)
        self.ax.set_ybound(-0.5, 0.5)
        self.ax.set_zbound(0.0, 0.5)

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        if limits is not None:
            self.ax.set_xlim3d([limits[0], limits[1]])
            self.ax.set_ylim3d([limits[2], limits[3]])
            self.ax.set_zlim3d([limits[4], limits[5]])

        self.limits = limits
       
        plt.ion()


def add(self, ob, readonly=False, display=True,
    jointaxes=True, jointlabels=False, eeframe=True, shadow=True, name=True, options=None):


    self.robots.append(
        RobotPlot(
            ob, self, readonly, display,
            jointaxes, jointlabels, eeframe, shadow, name, options))

    self.robots[len(self.robots) - 1].draw()


def step(self, dt=0.05):

    for robot in self.robots:
        robot.draw()

    self._set_axes_equal()


if __name__ == "__main__": None