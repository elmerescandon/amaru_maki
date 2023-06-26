from PyQt5.QtWidgets import *  
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from QSwitchControl import SwitchControl
import sys
import serial

from UI_design import UI_design
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from utils import * 
from utils_math import *

class ApplicationWindow(QMainWindow):   

    # Signals
    begin_calib = pyqtSignal(serial.Serial)
    begin_datarcv = pyqtSignal(serial.Serial)

    def __init__(self, *args, **kwargs):
        super(ApplicationWindow, self).__init__(*args,**kwargs)

        # Diseño de la interfaz
        UI_design(self) 
        
        # **********************************************************
        #       Inicializar variables y valores esenciales
        # **********************************************************
        # Configuración de Puerto Serial
        self.PORT = serial.Serial()

        # Variables de medición
        self.C1_MAX = 0.0
        self.C1_MIN = 0.0
        self.C1_RANGO = 0.0

        self.M1_MAX = 0.0
        self.M1_MIN = 0.0
        self.M1_RANGO = 0.0

        self.M2_MAX = 0.0
        self.M2_MIN = 0.0
        self.M2_RANGO = 0.0

        self.boton_iniciar.clicked.connect(self.INICIAR_COM) # Botón iniciar - Callback
        self.boton_envio.clicked.connect(self.envio_CB) # Botón Modo Envío - Callback 
        self.boton_calib.clicked.connect(self.calib_sensor_CB) # Botón Modo Calibración - Callback

        # **********************************************************
        #       Not done YET
        # **********************************************************

        # Configuraciones de gráfica
        self.canvas = MplCanvas(self, width=5, height=4, dpi=100)
        self.verticalLayout.addWidget(self.canvas)

        # Configuración del vector 
        self.n_data = 600
        self.data_count = 0
        self.data_joints = np.zeros((self.n_data, 6))
        self.xdata = list(range(self.n_data))

        # Update Plot
        self._plot_ref_1 = None
        self._plot_ref_2 = None
        self._plot_ref_3 = None

        self.update_plot()
        self.show()

        # Llamada a funciones de callback
        self.boton_calib_init.clicked.connect(self.init_calib_CB)
        self.boton_toggle_data.toggled.connect(self.recibir_datos_CB)

        # Clase Worker - Puerto Serial Comunicación
        self.thread = QThread()
        self.worker = Worker()
        self.worker.moveToThread(self.thread)   
        self.thread.start()

        # Conexiones Worker Serial
        self.begin_datarcv.connect(self.worker.run)
        self.begin_calib.connect(self.worker.calib)
        self.worker.progress_rcvdata.connect(self.write_text)

        # Clase Math Worker - Obtención de cinemática inversa
        self.thread_2 = QThread()
        self.worker_math = Worker_Math()
        self.worker_math.moveToThread(self.thread_2)
        self.thread_2.start()

        # Conexiones Math Worker
        self.worker.progress_listdata.connect(self.worker_math.get_joint)
        self.worker_math.progress_joints_data.connect(self.update_joints)

        # Setup a timer to trigger the redraw by calling update_plot.
        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_plot)
        

    # **********************************************************
    #               Funciones de callback y de uso
    # **********************************************************
    # Callback de botón que inicia la comunicación con Wearable
    def INICIAR_COM(self):
        self.PORT = serial.Serial(port = 'COM5', baudrate=115200,bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
        self.boton_iniciar.setEnabled(False)
        data = self.PORT.readline()
        data_s = data[:-1].decode('utf-8')
        print(data_s)
        if (data_s == "Error - IMU1" or data_s == "Error - IMU2" or data_s == "Error - IMU3"):
            print("Error con alguno de los IMUs: Reiniciar o Resetear")
            self.close()

    # Callback de botón que entra al modo Envío 
    def envio_CB(self):
        print("Cambiar a modo recibir datos")
        self.PORT.write("s2e".encode())
        data = self.PORT.readline()
        data_s = data[:-1].decode('utf-8')
        print(data_s)

    #  Callback de botón que entra al modo Calibración
    def calib_sensor_CB(self):
        print("Cambiar a modo calibración")
        self.PORT.write("s1e".encode())
        data = self.PORT.readline()
        data_s = data[:-1].decode('utf-8')
        print(data_s)

    # Callback de Toggle que inicia el envío de datos
    def recibir_datos_CB(self):
        estado = self.boton_toggle_data.checkState()
        if (estado):
             self.PORT.write("s21e".encode())
             self.begin_datarcv.emit(self.PORT)
             self.timer.start()
        else:
            self.PORT.write("s20e".encode())  
            self.timer.stop()

    # Callback para iniciar la calibración
    def init_calib_CB(self):
        self.PORT.write("s11e".encode())
        self.begin_calib.emit(self.PORT)  
        print("Iniciar Calibración")

    # Callback de Slot que escribe los datos leídos en pantalla
    def write_text(self, text):
        self.datosSerial.append(text)


    # Callback actualizar datos 
    def update_joints(self,q_joints):
        self.data_joints[self.data_count,:] = q_joints
        
        if (self.data_count == self.n_data-1):
            self.data_count = 100
            self.data_joints[0:100,:] = self.data_joints[500:600,:]
        else:
            self.data_count = self.data_count + 1
        


    def update_plot(self):
        if self._plot_ref_1 is None and self._plot_ref_2 is None and self._plot_ref_3 is None :
            plot_refs_1 = self.canvas.axes1.plot(self.xdata, self.data_joints[:,3])
            self.canvas.axes1.set(ylim=[-100,100],xlim=[0,600])

            plot_refs_2 = self.canvas.axes2.plot(self.xdata, self.data_joints[:,4])
            self.canvas.axes2.set(ylim=[-100,100],xlim=[0,600])

            plot_refs_3 = self.canvas.axes3.plot(self.xdata, self.data_joints[:,5])
            self.canvas.axes3.set(ylim=[-100,100],xlim=[0,600])

            self._plot_ref_1 = plot_refs_1[0]
            self._plot_ref_2 = plot_refs_2[0]
            self._plot_ref_3 = plot_refs_3[0]
        
        else:
            self._plot_ref_1.set_ydata(self.data_joints[:,3])
            self._plot_ref_2.set_ydata(self.data_joints[:,4])
            self._plot_ref_3.set_ydata(self.data_joints[:,5])

        self.canvas.draw()
        self.update_values()

    def update_values(self):

        q_joint_temp = self.data_joints[self.data_count-1,:]
        if (q_joint_temp[3] > self.C1_MAX):
            self.C1_MAX = q_joint_temp[3]
        if (q_joint_temp[3] < self.C1_MIN):
            self.C1_MIN = q_joint_temp[3]

        if (self.C1_RANGO < (abs(self.C1_MIN) + abs(self.C1_MAX))):
            self.C1_RANGO = np.round(abs(self.C1_MIN) + abs(self.C1_MAX),2)


        if (q_joint_temp[4] > self.M1_MAX):
            self.M1_MAX = q_joint_temp[4]
        if (q_joint_temp[4] < self.M1_MIN):
            self.M1_MIN = q_joint_temp[4]

        if (self.M1_RANGO < (abs(self.M1_MIN) + abs(self.M1_MAX))):
            self.M1_RANGO = np.round((abs(self.M1_MIN) + abs(self.M1_MAX)),2)


        if (q_joint_temp[5] > self.M2_MAX):
            self.M2_MAX = q_joint_temp[5]
        if (q_joint_temp[5] < self.M2_MIN):
            self.M2_MIN = q_joint_temp[5]

        if (self.M2_RANGO < (abs(self.M2_MIN) + abs(self.M2_MAX))):
            self.M2_RANGO = np.round((abs(self.M2_MIN) + abs(self.M2_MAX)),2)

        self.M1_MIN_VALUE.setText(str(self.M1_MIN))
        self.M2_MIN_VALUE.setText(str(self.M2_MIN))
        self.C1_MIN_VALUE.setText(str(self.C1_MIN))

        self.M1_MAX_VALUE.setText(str(self.M1_MAX))
        self.M2_MAX_VALUE.setText(str(self.M2_MAX))
        self.C1_MAX_VALUE.setText(str(self.C1_MAX))

        self.M1_RANGO_VALUE.setText(str(self.M1_RANGO))
        self.M2_RANGO_VALUE.setText(str(self.M2_RANGO))
        self.C1_RANGO_VALUE.setText(str(self.C1_RANGO))

    def closeEvent(self, event):
        self.PORT.close()
        print ("closing PyQtTest")

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    application = ApplicationWindow()
    
    application.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()