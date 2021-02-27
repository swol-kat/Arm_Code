import sys 
from PyQt5.QtWidgets import QDialog, QApplication, QPushButton, QVBoxLayout, QInputDialog, QLineEdit, QHBoxLayout, QLabel
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas 
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar 
import matplotlib.pyplot as plt 
import random
   
# main window 
# which inherits QDialog 
class Window(QDialog): 
       
    # constructor 
    def __init__(self, arm_comm, parent=None): 
        super(Window, self).__init__(parent) 

        self.setWindowTitle('it a kat')
        self.arm_comm = arm_comm
        # a figure instance to plot on 
        self.figure = plt.figure() 
   
        # this is the Canvas Widget that  
        # displays the 'figure'it takes the 
        # 'figure' instance as a parameter to __init__ 
        self.canvas = FigureCanvas(self.figure) 
   
        # this is the Navigation widget 
        # it takes the Canvas widget and a parent 
        self.toolbar = NavigationToolbar(self.canvas, self) 
        
        self.data = [[],[],[]]
        for i in range(0, 3):
            for j in range(0, 8):
                self.data[i].append(random.random() * 20 - 10)

        # Just some button connected to 'plot' method 
        self.button = QPushButton('Go To Position') 

        self.xyz_layout = QHBoxLayout()

        self.arm_input_pos = [0,0,0]

        self.line = []
        self.axisLabels = []

        axes = ['X:', 'Y:', 'Z:']

        for i in range(0,3):
            self.line.append(QLineEdit(self))
            self.axisLabels.append(QLabel(self))
            self.axisLabels[-1].setText(axes[i])
            self.xyz_layout.addWidget(self.axisLabels[-1])
            self.xyz_layout.addWidget(self.line[-1])
        self.xyz_layout.addWidget(self.button)

        # adding action to the button 
        self.button.clicked.connect(self.update_xyz) 
   
        # creating a Vertical Box layout 
        layout = QVBoxLayout() 
           
        # adding tool bar to the layout 
        layout.addWidget(self.toolbar) 
           
        # adding canvas to the layout 
        layout.addWidget(self.canvas)  

        layout.addLayout(self.xyz_layout)

        #add jog functionality

        #graph updating
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.plot)
        self.timer.start(1000)
           
        # setting layout to the main window 
        self.setLayout(layout) 
   
    def update_xyz(self):
        self.arm_input_pos[0] = float(self.line[0].text())
        self.arm_input_pos[1] = float(self.line[1].text())
        self.arm_input_pos[2] = float(self.line[2].text())
        #set arm target with x y z

    # action called by thte push button 
    def plot(self): 
        if self.arm_comm:
            while self.arm_comm.poll():
                arm_dict = self.arm_comm.recv()
                self.data[0] = arm_dict[joint_pos]['x']
                self.data[0] = arm_dict[joint_pos]['y']
                self.data[0] = arm_dict[joint_pos]['z']

        # clearing old figure 
        self.figure.clear() 
   
        # create an axis 
        ax = self.figure.add_subplot(111, projection = '3d') 
   
        # plot data 
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_zlim(-10, 10)
        
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

        ax.plot(self.data[0], self.data[1], self.data[2], '*-') 
   
        # refresh canvas 
        self.canvas.draw() 
        
        #send back the buttons we care about
        arm_out_dict = {}
        arm_out_dict['x'] = self.arm_input_pos[0]
        arm_out_dict['y'] = self.arm_input_pos[1]
        arm_out_dict['z'] = self.arm_input_pos[2]

        #other data out?

        if self.arm_comm:
            self.arm_comm.send(arm_out_dict)


    
        
   
# driver code 
def gui_worker(arm_comm):
       
    # creating apyqt5 application 
    app = QApplication(sys.argv) 
   
    # creating a window object 
    main = Window(arm_comm) 
       
    # showing the window 
    main.show() 
   
    # loop 
    sys.exit(app.exec_())

if __name__ == '__main__':
    gui_worker()