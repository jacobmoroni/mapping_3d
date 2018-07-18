import matplotlib.pyplot as plt
import numpy as np

class SelectStart:
    def __init__(self, line):
        self.line = line
        # self.xs = list(line.get_xdata())
        # self.ys = list(line.get_xdata())
        self.xs = [0,0]
        self.ys = [0,0]
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)
        self.node = 0
        self.start = [0,0]
        self.goal = [0,0]
    def __call__(self, event):
        # print('click', event)
        if event.inaxes!=self.line.axes: return
       
        if self.node == 0:
            self.xs[0] = event.xdata
            self.ys[0] = event.ydata
            self.start=[event.xdata,event.ydata]
            print "starting point selected: x= ",event.xdata," y= ", event.ydata 
            self.node = 1
        else:
            self.xs[1] = event.xdata
            self.ys[1] = event.ydata
            self.goal = [event.xdata,event.ydata]
            print "goal selected: x= ",event.xdata," y= ", event.ydata 
            self.node = 0
        self.line.set_data(self.xs, self.ys)
        self.line.figure.canvas.draw()
        return self.start,self.goal

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_title('click to select start and finish')
line, = ax.plot([0,0], [0,0],"xr")  # empty line
selector = SelectStart(line)
plt.show()
print selector.start
