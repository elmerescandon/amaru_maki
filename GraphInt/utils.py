from PyQt5.QtWidgets import *  
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from QSwitchControl import SwitchControl
import numpy as np
# Enter CMD arguments
import csv

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from utils import *
from matplotlib.axes._subplots import Axes
from utils_math import *

# Clase de Canvas para Figura en GUi
class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes1 = self.fig.add_subplot(311)
        self.axes2 = self.fig.add_subplot(312)
        self.axes3 = self.fig.add_subplot(313)
        super(MplCanvas, self).__init__(self.fig)
        
    

# Clase trabajadora como tarea adicional
class Worker(QObject):
    def __init__(self):
        super(Worker, self).__init__()
        self.fout = open('data.txt', 'w', newline='')
        self.o =csv.writer(self.fout)

    progress_rcvdata = pyqtSignal(str)
    progress_listdata = pyqtSignal(list)

    def run(self,PORT):
        count = 0
        self.PORT_w = PORT
        while (True):
            data = self.PORT_w.readline()
            data_s = data[:-1].decode('utf-8')
            if (data_s == "ENVIO:PAUSA"):
                break
            if (data_s == "ENVIO:ACITVAR"):
                continue

            data_substr = data_s.split(',') # Poner todos los numeros a str
            print(data_substr)
            # data_float = np.array(data_substr, dtype=np.float_)
            count = count + 1
            if count > 30:
                # if (data_float.size == 12):
                self.o.writerow(data_substr) 
                self.progress_rcvdata.emit(data_s)
                    # self.progress_listdata.emit(list(data_float))

    def calib(self,PORT):
        self.PORT_w = PORT
        while (True):
            data = self.PORT_w.readline()
            data_s = data[:-1].decode('utf-8')
            if (data_s == "CALIB:LISTO"):
                print("adios")
                break
            print(data_s)
            self.progress_rcvdata.emit(data_s)


class Worker_Math(QObject):

    progress_joints_data = pyqtSignal(np.ndarray)

    def __init__(self):
        super(Worker_Math,self).__init__()
        self.joint_prev = np.array([0.,0.,0. ,0.,0.,0.])
        self.joint_q = np.array([0.,0.,0. ,0.,0.,0.])
    
    def get_joint(self,quat_data):
        self.joint_prev = self.joint_q
        self.joint_q = get_joints(quat_data,self.joint_prev)

        if (np.linalg.norm(self.joint_prev) - np.linalg.norm(self.joint_q) < 5.0 ):
            self.progress_joints_data.emit(self.joint_q)
       
