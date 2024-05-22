# Import PyQt6 stuff
from PyQt6 import uic
from PyQt6.QtCore import QTimer, QCoreApplication, QDir
from PyQt6.QtWidgets import QWidget, QPushButton, QCheckBox, QRadioButton, QComboBox, \
                            QLineEdit, QToolButton, QSpinBox, QDoubleSpinBox, QMenuBar, \
                            QMessageBox, QFileDialog
# from PyQt6.QtGui import *

# Import other stuff
import os
from pathlib import Path
from matplotlib import pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import numpy as np
import json


# Load UI files
thispath = os.path.dirname(os.path.realpath(__file__)).replace("\\", "/")
rootpath = os.path.dirname(thispath).replace("\\", "/")
respath = f"{rootpath}/resources"
confpath = f"{rootpath}/config"
FormUI, WindowUI = uic.loadUiType(f"{respath}/mainwindow.ui")


class MainWindow(FormUI, WindowUI):
    done_loading = False
    busy = False
    wait_switch = False
    main_to = 1000  # ms
    temp_settings_file = f"{rootpath}/config/temp_settings.json"
    default_settings_file = f"{confpath}/PyTaper_default_settings.json"
    factory_settings_file = f"{confpath}/PyTaper_factory_settings.json"
    last_dir = "test conflict" #QDir.homePath()
    
    def __del__(self):
        print("closing all")
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.setupUi(self)
        self.show()
        self.initialize()
        self.config_ui()
        self.enable_controls()
        
        self.daq_init()
        
        self.done_loading = True
    
    # Configure UI
    def config_ui(self):
        # Sliders icons
        self.inoutIndSlider.setStyleSheet(
            f"QSlider:vertical {{ \
                min-width: 16px; \
            }} \
            \
            QSlider::groove:vertical {{ \
                width: 7px; \
                border: 1px solid #999999; \
                border-radius: 4px; \
            }} \
            \
            QSlider::handle:vertical {{ \
                image: url({respath}/flame_orange.png); \
                margin: -116px -6px; \
            }}"
        )
        
        self.brusherIndSlider.setStyleSheet(
            f"QSlider:horizontal {{ \
                min-height: 32px; \
            }} \
            \
            QSlider::groove:horizontal {{ \
                height: 10px; \
                border: 1px solid #999999; \
                border-radius: 4px; \
            }} \
            \
            QSlider::handle:horizontal {{ \
                image: url({respath}/flame_orange.png); \
                margin: -10px -76px; \
            }}"
        )
        
        self.pullLeftIndSlider.setStyleSheet(
            f"QSlider:horizontal {{ \
                min-height: 32px; \
            }} \
            \
            QSlider::groove:horizontal {{ \
                height: 10px; \
                border: 1px solid #999999; \
                border-radius: 4px; \
            }} \
            \
            QSlider::handle:horizontal {{ \
                image: url({respath}/arrow_left.png); \
                margin: -10px -136px; \
            }}"
        )
        
        self.pullRightIndSlider.setStyleSheet(
            f"QSlider:horizontal {{ \
                min-height: 32px; \
            }} \
            \
            QSlider::groove:horizontal {{ \
                height: 10px; \
                border: 1px solid #999999; \
                border-radius: 4px; \
            }} \
            \
            QSlider::handle:horizontal {{ \
                image: url({respath}/arrow_right.png); \
                margin: -10px -136px; \
            }}"
        )
        
        # Plots
        SMALL_SIZE = 7
        MEDIUM_SIZE = 8
        BIGGER_SIZE = 10
        plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
        plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
        plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
        plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
        plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
        plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
        plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title
        plt.rcParams['figure.constrained_layout.use'] = True

        self.figure_pow = plt.figure()
        self.graph_pow = FigureCanvas(self.figure_pow)
        self.graphToolbar_pow = NavigationToolbar(self.graph_pow, self)
        self.powGraph.addWidget(self.graphToolbar_pow)
        self.powGraph.addWidget(self.graph_pow)
        self.graph_pow_ax = self.figure_pow.add_subplot()
        self.graph_pow_ax.set_xlabel("Pulled Dist. (mm)")
        self.graph_pow_ax.set_ylabel("Transm. Power. (mW)")
        self.graph_pow_line, = self.graph_pow_ax.plot([0, 0], [0, 0])
        self.graph_pow_ax.grid(True)
        self.graph_pow.draw()
        
        self.figure_spec = plt.figure()
        self.graph_spec = FigureCanvas(self.figure_spec)
        self.graphToolbar_spec = NavigationToolbar(self.graph_spec, self)
        self.spectrumGraph.addWidget(self.graphToolbar_spec)
        self.spectrumGraph.addWidget(self.graph_spec)
        self.graph_spec_ax = self.figure_spec.add_subplot()
        self.graph_spec_img = self.graph_spec_ax.imshow([[0.0, 0.0, 0.0], [0.5, 0.5, 0.5], [1.0, 1.0, 1.0]], aspect="auto",
                                                        extent=(0.0, 1.0, 0.0, 1.0), interpolation='spline36', origin="lower")
        self.graph_spec_ax.set_xlabel("Pulled Dist. (mm)")
        self.graph_spec_ax.set_ylabel("Frequency (1/mm)")
        self.graph_spec_ax.grid(False)
        self.graph_spec.draw()
        
        self.figure_hz = plt.figure()
        self.graph_hz = FigureCanvas(self.figure_hz)
        self.graphToolbar_hz = NavigationToolbar(self.graph_hz, self)
        self.hotzoneGraph.addWidget(self.graphToolbar_hz)
        self.hotzoneGraph.addWidget(self.graph_hz)
        self.graph_hz_ax = self.figure_hz.add_subplot()
        self.graph_hz_line, = self.graph_hz_ax.plot([0, 0], [0, 0])
        self.graph_hz_ax.set_xlabel("Pulled Dist. (mm)")
        self.graph_hz_ax.set_ylabel("Hotzone Size (mm)")
        self.graph_hz_ax.grid(True)
        self.graph_hz.draw()
          
        # Timers
        self.mainLoop_timer = QTimer()
        self.mainLoop_timer.setInterval(self.main_to)
        self.mainLoop_timer.timeout.connect(self.mainLoop)
        self.mainLoop_timer.start()
        
        self.initLoop_timer = QTimer()
        self.initLoop_timer.setInterval(self.main_to)
        self.initLoop_timer.timeout.connect(self.initLoop)
        
        self.flameholdLoop_timer = QTimer()
        self.flameholdLoop_timer.setInterval(self.main_to)
        self.flameholdLoop_timer.timeout.connect(self.flameholdLoop)
        
        self.pullLoop_timer = QTimer()
        self.pullLoop_timer.setInterval(self.main_to)
        self.pullLoop_timer.timeout.connect(self.pullLoop)
        
        self.go2zeroLoop_timer = QTimer()
        self.go2zeroLoop_timer.setInterval(self.main_to)
        self.go2zeroLoop_timer.timeout.connect(self.go2zeroLoop)
        
        # DAQ connections
        self.srateSpin.valueChanged.connect(self.daq_init)
        self.daqChannelCombo.currentIndexChanged.connect(self.daq_init)
        self.daqConfCombo.currentIndexChanged.connect(self.daq_init)
        self.daqClockCombo.currentIndexChanged.connect(self.daq_init)
        self.daqminvSpin.valueChanged.connect(self.daq_init)
        self.daqmaxvSpin.valueChanged.connect(self.daq_init)
        self.set0powBut.clicked.connect(self.set_ref_power)
        
        # Motors connections
        self.resetmotorsBut.clicked.connect(self.init_motors)
        self.gotozeroBut.clicked.connect(self.go2start)
        self.startBut.clicked.connect(self.start_pulling)
        self.stopBut.clicked.connect(self.stop_pulling)
        self.cleaveBut.clicked.connect(self.cleave)
        self.emerBut.clicked.connect(self.stop_all_motors)
        self.flameIOPosIndSpin.valueChanged.connect(self.fio_move2)
        self.brusherPosIndSpin.valueChanged.connect(self.fb_move2)
        self.leftPosIndSpin.valueChanged.connect(self.lp_move2)
        self.rightPosIndSpin.valueChanged.connect(self.rp_move2)
        self.enablemanualCheck.clicked.connect(self.toggle_manual_control)
        
        # Taper params connections
        self.distPriorRadio.clicked.connect(self.recalc_params)
        self.diamPriorRadio.clicked.connect(self.recalc_params)
        self.d0Spin.valueChanged.connect(self.recalc_params)
        self.x0Spin.valueChanged.connect(self.recalc_params)
        self.l0Spin.valueChanged.connect(self.recalc_params)
        self.dwSpin.valueChanged.connect(self.recalc_params)
        self.fSizeSpin.valueChanged.connect(self.recalc_params)
        self.alphaSpin.valueChanged.connect(self.recalc_params)
        self.loadHZBut.clicked.connect(self.load_hotzone)
        
         # Menu connections
        self.actionSave_Data.triggered.connect(self.action_save_data)
        self.actionLoad_Default_Settings.triggered.connect(self.action_load_def_settings)
        self.actionSet_Current_as_Default.triggered.connect(self.action_set_def_settings)
        self.actionLoad_Settings.triggered.connect(self.action_load_settings)
        self.actionSave_Settings.triggered.connect(self.action_save_settings)
        self.actionReset_to_Factory.triggered.connect(self.action_factory_settings)
        self.actionExit.triggered.connect(self.action_exit)
        self.actionAbout.triggered.connect(self.action_about)
        self.actionAbout_Qt.triggered.connect(self.action_about_qt)
    
        # Settings
        Path(confpath).mkdir(parents=True, exist_ok=True)
        if not os.path.isfile(self.default_settings_file):
            self.save_settings(self.default_settings_file)
            
        if not os.path.isfile(self.factory_settings_file):
            self.save_settings(self.factory_settings_file)
        else:
            self.load_default_settings()
            
    # General functions
    def initialize(self):
        self.tabWidget.setCurrentIndex(0)
        self.reset_pull_stats()
        self.recalc_params()        

    def save_settings(self, filename):
        if filename != "":
            directory = os.path.dirname(os.path.realpath(filename)).replace("\\", "/")
            Path(directory).mkdir(parents=True, exist_ok=True)
            
            settings_dict = {}
            settings_dict["__lastdir__"] = self.last_dir
            for w in self.findChildren(QSpinBox):
                settings_dict[w.objectName()] = w.value()
            for w in self.findChildren(QDoubleSpinBox):
                settings_dict[w.objectName()] = w.value()
            for w in self.findChildren(QCheckBox):
                settings_dict[w.objectName()] = w.isChecked()
            for w in self.findChildren(QRadioButton):
                settings_dict[w.objectName()] = w.isChecked()
            for w in self.findChildren(QComboBox):
                settings_dict[w.objectName()] = w.currentIndex()
            for w in self.findChildren(QLineEdit):
                settings_dict[w.objectName()] = w.text()
                
            json.dump(settings_dict, open(filename, "w"))
    
    def load_settings(self, filename):
        if filename != "":
            directory = os.path.dirname(os.path.realpath(filename)).replace("\\", "/")
            Path(directory).mkdir(parents=True, exist_ok=True)

            settings_dict = json.load(open(filename, "r"))
            if "__lastdir__" in settings_dict:
                self.last_dir = settings_dict["__lastdir__"]
            for key in settings_dict:
                if key[:2] != "__" and key[-2:] != "__":
                    try:
                        w = self.findChild(QWidget, key)
                        if "Spin" in key:
                            w.setValue(settings_dict[key])
                        if "Check" in key:
                            # For safety, never enable manual control
                            if key != "enablemanualCheck":
                                w.setChecked(settings_dict[key])
                        if "Radio" in key:
                            w.setChecked(settings_dict[key])
                        if "Combo" in key:
                            w.setCurrentIndex(settings_dict[key])
                        if "Text" in key:
                            w.setText(settings_dict[key])
                    except:
                        pass
            print(self.last_dir)
    
    def load_default_settings(self):
        self.load_settings(self.default_settings_file)
    
    def enable_controls(self):
        widgets = self.findChildren(QWidget)
        for i in range(len(widgets)):
            widgets[i].setEnabled(True)
        self.stopBut.setEnabled(False)
        
    def disable_controls(self):
        lines = self.findChildren(QLineEdit)
        for i in range(len(lines)):
            if "Text" in lines[i].objectName():
                lines[i].setEnabled(False)

        buttons = self.findChildren(QPushButton)
        for i in range(len(buttons)):
            buttons[i].setEnabled(False)
        
        tbuttons = self.findChildren(QToolButton)
        for i in range(len(tbuttons)):
            tbuttons[i].setEnabled(False)
        
        checks = self.findChildren(QCheckBox)
        for i in range(len(checks)):
            checks[i].setEnabled(False)
        
        radios = self.findChildren(QRadioButton)
        for i in range(len(radios)):
            radios[i].setEnabled(False)
        
        spins = self.findChildren(QSpinBox)
        for i in range(len(spins)):
            spins[i].setEnabled(False)
            if spins[i].isReadOnly():
                spins[i].setEnabled(True)
            
        dspins= self.findChildren(QDoubleSpinBox)
        for i in range(len(dspins)):
            dspins[i].setEnabled(False)
            if dspins[i].isReadOnly():
                dspins[i].setEnabled(True)
            
        menus = self.findChildren(QMenuBar)
        for i in range(len(menus)):
            menus[i].setEnabled(False)
        
        combos = self.findChildren(QComboBox)
        for i in range(len(combos)):
            combos[i].setEnabled(False)
        
        self.emerBut.setEnabled(True)
        self.cologradSpin.setEnabled(True)
        
    def reset_pull_stats(self):
        print("Pull stats reset")

    # Timer functions
    def mainLoop(self):
        print("main loop")
    
    def initLoop(self):
        print("init loop")
    
    def flameholdLoop(self):
        print("flame hold loop")
    
    def pullLoop(self):
        print("pull loop")
    
    def go2zeroLoop(self):
        print("go2zero loop")
    
    # DAQ functions
    def daq_init(self):
        print("inited DAQ")
    
    def set_ref_power(self):
        print("set ref. power")
        
    # Motors functions
    def toggle_manual_control(self):
        self.flameIOPosIndSpin.setReadOnly(not self.enablemanualCheck.isChecked())
        self.brusherPosIndSpin.setReadOnly(not self.enablemanualCheck.isChecked())
        self.leftPosIndSpin.setReadOnly(not self.enablemanualCheck.isChecked())
        self.rightPosIndSpin.setReadOnly(not self.enablemanualCheck.isChecked())
        
        if self.enablemanualCheck.isChecked():
            buttons = QSpinBox.ButtonSymbols.UpDownArrows
        else:
            buttons = QSpinBox.ButtonSymbols.NoButtons

        self.flameIOPosIndSpin.setButtonSymbols(buttons)
        self.brusherPosIndSpin.setButtonSymbols(buttons)
        self.leftPosIndSpin.setButtonSymbols(buttons)
        self.rightPosIndSpin.setButtonSymbols(buttons)
        
    def fb_init(self):
        print("inited flame brusher")
        
    def fio_init(self):
        print("inited flame in/out")
        
    def lp_init(self):
        print("inited left puller")
        
    def rp_init(self):
        print("inited right puller")
    
    def fb_move2(self):
        print("moving flame brusher")
        
    def fio_move2(self):
        print("moving flame in/out")
        
    def lp_move2(self):
        print("moving left puller")
        
    def rp_move2(self):
        print("moving right puller")
        
    def stop_all_motors(self):
        print("stopped all motors")
        
    def init_motors(self):
        print("inited motors")
        self.fb_init()
        self.fio_init()
        self.lp_init()
        self.rp_init()
        
    def go2start(self):
        print("going to start")
        
    def start_pulling(self):
        print("started pulling")
        
    def stop_pulling(self):
        print("stopped pulling")
        
    def cleave(self):
        print("cleaved")
        
    def set_indicators(self):
        print("indicators set")
    
    # Taper params functions
    def recalc_params(self):
        alpha = self.alphaSpin.value()
        if np.abs(alpha) < 1e-15: alpha = 1e-15
        
        x0 = self.x0Spin.value()
        dw = self.dwSpin.value()
        d0 = self.d0Spin.value()
        l0 = self.l0Spin.value()
        
        z0 = (1 - alpha)*x0/2
        lw = l0 + alpha*x0

        if self.distPriorRadio.isChecked():
            rw = (d0/2)*((1 + alpha*x0/l0)**(-1/(2*alpha)))  # Explodes for alpha = 0
            self.dwSpin.setValue(2*rw)
        elif self.diamPriorRadio.isChecked():
            x0 = 2*(pow(((d0/2)/(dw/2)),(2*alpha)) - 1)*l0/(2*alpha)
            self.x0Spin.setValue(x0)

        self.setTransLengthSpin.setValue(z0)
        self.setWaistLengthSpin.setValue(lw)
        self.timeleftSpin.setValue(x0/(2.0*self.pullerPullVelSpin.value()))
        
    def load_hotzone(self):
        print("loaded hotzone profile")
        
    # Menu functions
    def action_save_data(self):
        print("saved data")
        
    def action_load_def_settings(self):
        self.load_default_settings()
        
    def action_set_def_settings(self):
        self.save_settings(self.default_settings_file)
        
    def action_load_settings(self):
        file = QFileDialog.getOpenFileName(self, "Load settings", self.last_dir, "json files (*.json)")
        filename = file[0]
        
        self.last_dir = os.path.dirname(os.path.realpath(filename)).replace("\\", "/")
       
        self.load_settings(filename)
        
        self.statusBar.showMessage(f"Settings loaded")
        
    def action_save_settings(self):
        file = QFileDialog.getSaveFileName(self, "Save settings", self.last_dir, "json files (*.json)")
        filename = file[0]
        
        self.last_dir = os.path.dirname(os.path.realpath(filename)).replace("\\", "/")
       
        self.save_settings(filename)
        
        self.statusBar.showMessage(f"Settings saved")
        
    def action_factory_settings(self):
        self.load_settings(self.factory_settings_file)
    
    def action_exit(self):
        QCoreApplication.quit()
        
    def action_about(self):
        aboutText = \
            "<p><b>LPD Taper Pulling Software - PyTaper v1.0</b></p>" +\
            "<p>This software uses PyQt6, NI DAQmx API, and ThorLabs Kinesis API.</p>" +\
            "<p>Copyright &copy; 2017-2024</p>" +\
            "<p>By Paulo Jarschel</p>"

        QMessageBox.about(self, "About", aboutText)
    
    def action_about_qt(self):
        QMessageBox.aboutQt(self, "About Qt")

        
    