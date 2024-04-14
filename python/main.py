import serial
import time
from inputs import get_gamepad
import threading
import json
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QWidget, QFrame
from PyQt5.QtGui import QPixmap, QPainter, QPen, QColor, QPolygonF
from PyQt5.QtCore import QPointF, QTimer
import math

portsList = []

class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def read(self): # return the buttons/triggers that you care about in this methode
        thisdict = dict(
            LeftJoystickX   = self.LeftJoystickX,
            #LeftJoystickY   = self.LeftJoystickY,
            #RightJoystickX  = self.RightJoystickX,
            RightJoystickY  = self.RightJoystickY,
            LeftTrigger     = self.LeftTrigger,
            RightTrigger    = self.RightTrigger,
            #LeftBumper      = self.LeftBumper,
            #RightBumper     = self.RightBumper,
            AButton         = self.A,
            #BButton         = self.B,
            #XButton         = self.X,
            #YButton         = self.Y,
            #LeftThumb       = self.LeftThumb,
            #RightThumb      = self.RightThumb,
            #LeftDpad        = self.LeftDPad,
            #RightDpad       = self.RightDPad,
            #UpDpad          = self.UpDPad,
            #DownDpad        = self.DownDPad,
            BackButton       = self.Back,
            #BackStart       = self.Start,
        )
        return thisdict

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state #previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state
            time.sleep(0.01)

class Mainwindow(QMainWindow):

    def __init__(self):
        super().__init__()

        # Set up the initial parameters of the window
        self.setWindowTitle("Positioning the camera crane")
        self.setGeometry(1000, 100, 800, 1400)
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.canvas = QLabel()
        self.layout.addWidget(self.canvas)

        self.rotation_angle = 180
        self.tilt_angle = -60
        self.arm_length = 50
        self.telescope_length = 120
        self.telescope_elongation = 2

        self.gyrox_position = 0
        self.gyroy_position = 0
        self.gyroz_position = 0
        self.full_extend_state = 0
        self.full_retract_state = 0

        self.active_robot = False
        self.last_activation_state = False
        self.last_AButton_state = False

        self.activation_button = QPushButton("Desactivated", clicked=self.activation)
        self.layout.addWidget(self.activation_button)

        self.A_button = QPushButton("AButton", clicked=self.record_init_gyro)
        self.layout.addWidget(self.A_button)

        self.frame = QFrame()
        self.layout.addWidget(self.frame)
        self.layout_controle = QVBoxLayout(self.frame)

        self.buttons()

        self.base_triangle = QPixmap(1000, 600)
        self.simulation()

        self.init_gyro_values = {'GyroX': 0.0, 'GyroY': 0.0, 'GyroZ': 0.0}

        self.comInit()          #Set up the Serial communication
        self.periodicSetUp()    #will call the peiodic function at 100Hz

        self.update_button()
        self.state_update()
        self.state_infos()

        self.sender()
        self.receiver()

    def comInit(self):
        self.ser = serial.Serial("COM13", baudrate=115200,
                            timeout=2.5,
                            parity=serial.PARITY_NONE,
                            bytesize=serial.EIGHTBITS,
                            stopbits=serial.STOPBITS_ONE
                            )
        self.joy = XboxController()
        self.feedback = dict (
            LeftJoystickX   = 0.0,
            RightJoystickY  = 0.0,
            LeftTrigger     = 0.0,
            RightTrigger    = 0.0,
            StartButton     = 0,
            BackButton      = 0,
            Moteur1         = 1500,
            Moteur2         = 1500,
            Moteur3         = 1500,
            LimitIn         = 1,
            LimitOut        = 1,
            GyroX           = 0.0,
            GyroY           = 0.0,
            GyroZ           = 0.0
        )

    def periodicSetUp(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.receiver)
        self.timer.timeout.connect(self.sender)
        self.timer.timeout.connect(self.update_button)
        self.timer.timeout.connect(self.state_infos)
        self.timer.start(50)    # 50ms interval

    def receiver(self):
        print("Trying to receive data")
        if self.ser.isOpen():
            try:
                incoming = self.ser.readline().decode("utf-8")
                self.ser.reset_input_buffer()
                self.feedback = json.loads(incoming)
                self.state_update()
                print("got:", incoming)
            except Exception as e:
                print(e)
                pass
        print("Data received!")

    def sender(self):
        print("Trying to send data")
        # Activation button state
        dict = self.joy.read()
        dict.update({'BackButtonAct':self.active_robot})

        json1 = json.dumps(dict)

        if self.ser.isOpen():
            data_js = json1.encode('ascii')
            print(data_js)
            self.ser.reset_output_buffer()
            self.ser.write(json1.encode('ascii'))
        else:
            print("ERROR : Cannot open serial port")
            print("Data sent")

    def record_init_gyro(self):
        self.init_gyro_values['GyroX'] = self.gyrox_position
        self.init_gyro_values['GyroY'] = self.gyroy_position
        self.init_gyro_values['GyroZ'] = self.gyroz_position

    def state_update(self):
        self.gyrox_position = self.feedback.get('GyroX')
        self.gyroy_position = self.feedback.get('GyroY')
        self.gyroz_position = self.feedback.get('GyroZ')
        self.full_retract_state = self.feedback.get('LimitIn')
        self.full_extend_state = self.feedback.get('LimitOut')

    def state_infos(self):
        print("Updating UI")
        if not hasattr(self, 'gyrox'):
            self.gyrox = QLabel()
            self.layout.addWidget(self.gyrox)
        self.real_gyrox = self.gyrox_position - self.init_gyro_values['GyroX']
        self.gyrox.setText("X position : " + str(self.real_gyrox))

        if not hasattr(self, 'gyroy'):
            self.gyroy = QLabel()
            self.layout.addWidget(self.gyroy)
        self.real_gyroy = self.gyroy_position - self.init_gyro_values['GyroY']
        self.gyroy.setText("Y position : " + str(self.real_gyroy))

        if not hasattr(self, 'gyroz'):
            self.gyroz = QLabel()
            self.layout.addWidget(self.gyroz)
        self.real_gyroz = self.gyroz_position - self.init_gyro_values['GyroZ']
        self.gyroz.setText("Z position : " + str(self.real_gyroz))

        if not hasattr(self, 'full_extend'):
            self.full_extend = QLabel()
            self.layout.addWidget(self.full_extend)
        self.full_extend.setText("Telescope full extend : " + str(self.full_extend_state))

        if not hasattr(self, 'full_retract'):
            self.full_retract = QLabel()
            self.layout.addWidget(self.full_retract)
        self.full_retract.setText("Telescope full retract : " + str(self.full_retract_state))
        print("UI updated!")


    # Update the buttons state w
    # ith the controller and execute a function
    def update_button(self):
        print("Starting the button update")
        # LeftTrigger retract the telescope and the RightTrigger extend it
        if self.joy.read().get('LeftTrigger') is not None and self.joy.read().get('RightTrigger') is not None:
            if self.joy.read().get('LeftTrigger') > 0.1:
                self.retract_telescope()
            elif self.joy.read().get('RightTrigger') > 0.1:
                self.extend_telescope()
        # BackButton activated and desactivated the robot
        if self.joy.read().get('BackButton') != self.last_activation_state:
            if self.joy.read().get('BackButton') == True:
                self.last_activation_state = self.joy.read().get('BackButton')
                self.activation()
            else:
                self.last_activation_state = self.joy.read().get('BackButton')
        if self.joy.read().get('AButton') != self.last_AButton_state:
            if self.joy.read().get('AButton') == True:
                self.last_AButton_state = self.joy.read().get('AButton')
                self.record_init_gyro()
            else:
                self.last_activation_state = self.joy.read().get('BackButton')
        # LeftJoyStickX rotates the base in the positive direction when in the left position
        # and in negative direction when in the the right position
        if self.joy.read().get('LeftJoystickX') < -0.1:
            self.positive_rotation()
        if self.joy.read().get('LeftJoystickX') > 0.1:
            self.negative_rotation()
        # RightJoyStickY tilt up the arm when in the up position
        # and tilt down the arm when in the down position
        if self.joy.read().get('RightJoystickY') > 0.1:
            self.tilt_up()
        if self.joy.read().get('RightJoystickY') < -0.1:
            self.tilt_down()
        
        print("Button update done!")

    # General activation of the robot
    def activation(self):
        self.active_robot = not self.active_robot
        if self.active_robot:
            self.activation_button.setText("Activated")
            self.activation_button.setStyleSheet("background-color: green; color: white;")
        else:
            self.activation_button.setText("Desactivated")
            self.activation_button.setStyleSheet("background-color: red; color: white;")

    # Base rotation
    def positive_rotation(self):
        if self.active_robot:
            self.rotation_angle += 10
            self.simulation()
    def negative_rotation(self):
        if self.active_robot:
            self.rotation_angle -= 10
            self.simulation()

    # Definition of the UI's buttons
    def buttons(self):
        buttons = [
            ("Positive rotation", self.positive_rotation),
            ("Negative rotation", self.negative_rotation),
            ("Tilt up", self.tilt_up),
            ("Tilt down", self.tilt_down),
            ("Retract telescope", self.extend_telescope),
            ("Take out telescope", self.retract_telescope),
            #("Avancer contrepoids", self.contrepoids_pos),
            #("Reculer contrepoids", self.contrepoids_neg),
            #("Fermeture", self.fermeture)
        ]

        for texte, fonction in buttons:
            bouton = QPushButton(texte, clicked=fonction)
            self.layout_controle.addWidget(bouton)

    # Tilt of the arm
    def tilt_up(self):
        if self.active_robot:
            self.tilt_angle += 3
            self.simulation()
    def tilt_down(self):
        if self.active_robot:
            self.tilt_angle -= 3
            self.simulation()

    # Extension of the telescope
    def extend_telescope(self):
        if self.active_robot:
            self.telescope_length = self.telescope_length + self.telescope_elongation
            self.simulation()
    def retract_telescope(self):
        if self.active_robot:
            self.telescope_length = self.telescope_length - self.telescope_elongation
            self.simulation()

    # Drawing and controle of the simulation in the UI
    def simulation(self):
        self.base_triangle.fill(QColor("white"))

        painter = QPainter(self.base_triangle)
        painter.setPen(QPen(QColor("black")))

        # Drawing side view frame
        painter.drawLine(20, 30, 20, 90)
        painter.drawText(16, 23, "Y")

        painter.drawLine(20, 90, 80, 90)
        painter.drawText(90, 93, "X")

        # Drawing upperview frame
        painter.drawLine(895, 80, 895, 20)
        painter.drawText(890, 105, "Z")

        painter.drawLine(895, 20, 955, 20)
        painter.drawText(965, 29, "X")

        # Drawing the triangular base
        base_coords = [QPointF(200, 400), QPointF(170, 460), QPointF(230, 460)]
        painter.setBrush(QColor("gray"))
        painter.drawPolygon(QPolygonF(base_coords))

        # Calculation of the angle of drawn lines
        x1 = int(150 + self.arm_length)
        y1 = int(350 - self.arm_length)

        x2 = int(
            x1 + (self.arm_length + self.telescope_length) * math.cos(math.radians(90 + self.tilt_angle)))
        y2 = int(
            y1 - (self.arm_length + self.telescope_length) * math.sin(math.radians(90 + self.tilt_angle)))

        x4 = int(x1 - 75 * math.cos(math.radians(90 + self.tilt_angle)))
        y4 = int(y1 + 75 * math.sin(math.radians(90 + self.tilt_angle)))

        xd = int(750 - 100 * math.cos(math.radians(self.rotation_angle)))
        yd = int(300 + 100 * math.sin(math.radians(self.rotation_angle)))

        # Drawing lines
        painter.setPen(QPen(QColor("black"), 5))
        painter.drawLine(QPointF(200, 430), QPointF(x1, y1))
        painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))
        #painter.drawLine(QPointF(x1, y1), QPointF(x3, y3))
        painter.drawLine(QPointF(x1, y1), QPointF(x4, y4))

        # Drawing ellipses
        painter.setBrush(QColor("blue"))
        painter.drawEllipse(QPointF(x2, y2), 13, 13)

        # Drawing the upper view
        painter.setPen(QPen(QColor("black"), 2))
        painter.setBrush(QColor("white"))
        painter.drawEllipse(QPointF(750, 300), 100, 100)
        painter.drawLine(QPointF(750, 300), QPointF(xd, yd))

        # Writing title int the pixmap
        painter.drawText(175, 150, "Vue de côté")
        painter.drawText(685, 150, "Vue de dessus")

        painter.end()

        self.canvas.setPixmap(self.base_triangle)
        self.canvas.update()


if __name__ == '__main__':

    app = QApplication(sys.argv)
    window = Mainwindow()
    window.show()
    sys.exit(app.exec_())