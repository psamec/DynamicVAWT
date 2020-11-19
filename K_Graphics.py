# Import what matters
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import matplotlib.animation as animation


# Initialize Graphics
class Dashboard() :
    def __init__(self, wing_width, number_of_branches, arm_length, n_steps, max_power = 200):

        self.plt = plt
        self.plt.ion()
        self.fig = plt.figure()
        self.gs=GridSpec(4,2) # 3 rows, 2 columns
        self.animation = animation
        self.number_of_branches = number_of_branches
        self.arm_length = arm_length
        self.a = wing_width / 3
        self.n_steps = n_steps


        self.TorqueSubPlot = self.fig.add_subplot(self.gs[0,:])
        self.TorqueSubPlot.set(xlim=(0,n_steps), ylim=(0, max_power))

        self.TurbineSubPlot=self.fig.add_subplot(self.gs[1:,:-1]) # used to be gs[:,0]) #  First column spans all rows
        self.TurbineSubPlot.set_axis = ('equal', 'box')
        self.TurbineSubPlot.set(xlim=(-self.arm_length*1.1,self.arm_length*1.1),
                                ylim=(-self.arm_length*1.1,self.arm_length*1.1))
        self.TurbineSubPlot.set_aspect(1)
        self.TurbineSubPlot.set_title('Full Turbine')

        self.WingSubPlots = []

        for i in range(self.number_of_branches) :
            self.WingSubPlots.append(self.fig.add_subplot(self.gs[-i-1, -1])) # used to be gs[i, 1])) # First row, second column
            self.WingSubPlots[i].set(xlim=(-self.a*3, self.a*3), ylim=(-self.a, self.a))
            title = "Wing " # +str(i+1) no longer putting the wing numbrs now that we have coloring
            self.WingSubPlots[i].set_title(title)
            self.WingSubPlots[i].set_axis = ('equal')

        self.fig.savefig('gridspec.png')
        self.fig.tight_layout()

    def ShowFrame(self):
        self.plt.show()
        self.plt.pause(0.02)

    def ShowEnd(self):
        self.plt.ioff()
        self.plt.show()