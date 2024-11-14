# Import PyQt6 stuff
from PyQt6 import uic
from PyQt6.QtCore import Qt, QTimer, QCoreApplication, QDir, pyqtSignal, QObject, QEvent
from PyQt6.QtWidgets import QWidget, QPushButton, QCheckBox, QRadioButton, QComboBox, \
                            QLineEdit, QToolButton, QSpinBox, QDoubleSpinBox, QMenuBar, \
                            QMessageBox, QFileDialog, QLabel, QMenu
from PyQt6.QtGui import QPixmap

# Import other stuff
import os
import shutil
from pathlib import Path
from matplotlib import pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import numpy as np
import time
from datetime import datetime
import json
import cv2

# Import taper pulling classes
from TaperPulling.TaperShape import TaperShape
from TaperPulling.TaperPullingSim import TaperPullingSim
from TaperPulling.TaperPullingCore import TaperPullingCore
from TaperPulling.TaperPullingData import TaperPullingData
from TaperPulling.TaperPullingMotors import GenericTLMotor

# Load UI files
thispath = os.path.dirname(os.path.realpath(__file__)).replace("\\", "/")
rootpath = os.path.dirname(thispath).replace("\\", "/")
respath = f"{rootpath}/resources"
confpath = f"{rootpath}/config"
FormUI, WindowUI = uic.loadUiType(f"{respath}/mainwindow.ui")


# Status LEDs event filter (clickable label)
class LabelEventFilter(QObject):
    clicked = pyqtSignal()
    right_clicked = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
    def eventFilter(self, obj, event):
        if event.type() == event.Type.MouseButtonPress:
            if event.button() == Qt.MouseButton.RightButton:
                self.right_clicked.emit()
            elif event.button() == Qt.MouseButton.LeftButton:
                self.clicked.emit()
        return False



# Main class
class MainWindow(FormUI, WindowUI):
    # Taper classes objects
    shape = None
    simultaion = None
    core = None
    data = None
    
    # General variables
    max_tpts = 10000
    max_spectra = 1000
    temp_settings_file = f"{rootpath}/config/temp_settings.json"
    default_settings_file = f"{confpath}/PyTaper_default_settings.json"
    factory_settings_file = f"{confpath}/PyTaper_factory_settings.json"
    last_dir = QDir.homePath()
    
    # Flow control variables
    done_loading = False
    going_2start = False
    go2start_busy = False
    init_busy = False
    busy = False
    pull_busy = False
    daq_busy = False
    initLoop_active = False
    init_motors_tries = 10
    init_motors_i = 0
    wait_switch = False
    main_to = 100  # ms
    
    # Fabrication variables
    hz_function = np.array([[0.0, 1.0], [1.0, 1.0]])
    profile = np.array([[0.0, 1.0], [1.0, 1.0]])
    realish_profile = np.array([[0.0, 1.0], [1.0, 1.0]])
    min_hz = 0.0
    total_to_pull = 0.0
    adiab_angles = np.array([[0.0, 1.0], [1.0, 1.0]])
    profile_angles = np.array([[0.0, 1.0], [1.0, 1.0]])
    hz_sweep_n = 0
    rhz_array = []
    rhz_x_array = []
    transm_array = np.zeros((max_tpts, 2))
    spectrogram_data = np.zeros((2,2))
    transm_i = 0
    profile_mode = "Standard"  # Can be Standard, Optimized or Loaded
    
    def __del__(self):
        print("Closing all")
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Initialize taper classes
        self.shape = TaperShape()
        self.simultaion = TaperPullingSim()
        self.core = TaperPullingCore()
        self.data = TaperPullingData()
        
        self.setupUi(self)
        self.show()
        self.config_ui()
        self.initialize()
        self.enable_controls()
        self.toggle_manual_control()
        self.toggle_fiber_defaults()
        
        self.done_loading = True
        
    def closeEvent(self, event):
        reply = QMessageBox.question(self, 'Exiting...',
            "Are you sure to exit?", QMessageBox.StandardButton.Yes, QMessageBox.StandardButton.No)

        if reply == QMessageBox.StandardButton.Yes:
            self.pullLoop_timer.stop()
            self.mainLoop_timer.stop()
            self.initLoop_timer.stop()
            self.go2start_timer.stop()
            self.calc_timer.stop()
            self.data.stop_monitor()
            self.data.close()
            self.core.stop_pulling()
            self.core.close()
            self.core.motors.close()
            event.accept()
        else:
            event.ignore()
    
    # Function to add clicking functionality to LEDs
    def make_led_clickable(self, label: QLabel, motor: GenericTLMotor):
        # Create and install event filter
        event_filter = LabelEventFilter(label)
        label.installEventFilter(event_filter)
        
        # # Enable context menu
        label.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        
        # Add right-click menu functionality
        def show_context_menu(position):
            context_menu = QMenu(label)
            action1 = context_menu.addAction("Re-initialize (if initialized)")
            action2 = context_menu.addAction("Force home")
            
            action = context_menu.exec(label.mapToGlobal(position))
            
            if action == action1:
                motor.close()
                self.init_motors()
            
            if action == action2:
                if motor.name == "Brusher":
                    if self.core.motors.left_puller.danger_zone():
                        self.core.motors.left_puller.go_to(self.core.motors.left_puller.safe_range[1])
                    if self.core.motors.right_puller.danger_zone():
                        self.core.motors.right_puller.go_to(self.core.motors.right_puller.safe_range[1])
                motor.home(True)
        
        label.customContextMenuRequested.connect(show_context_menu)
        
        return event_filter
    
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
        
        # Colormaps
        for cmap in plt.colormaps():
            if not "_r" in cmap:
                self.cmapCombo.addItem(cmap)
        self.cmapCombo.setCurrentText('PRGn')
        
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
        # self.powGraph.addWidget(self.graphToolbar_pow)
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
        # self.spectrumGraph.addWidget(self.graphToolbar_spec)
        self.spectrumGraph.addWidget(self.graph_spec)
        self.graph_spec_ax = self.figure_spec.add_subplot()
        self.graph_spec_img = self.graph_spec_ax.imshow(np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]).T, aspect="auto",
                                                        extent=(0.0, 1.0, 0.0, 1.0), origin="lower", cmap='PRGn', interpolation='none')
        self.graph_spec_ax.set_xlabel("Pulled Dist. (mm)")
        self.graph_spec_ax.set_ylabel("Frequency (per µm pulled)")
        self.graph_spec_ax.grid(False)
        self.graph_spec.draw()
        
        self.figure_hz = plt.figure()
        self.graph_hz = FigureCanvas(self.figure_hz)
        self.graphToolbar_hz = NavigationToolbar(self.graph_hz, self)
        # self.hotzoneGraph.addWidget(self.graphToolbar_hz)
        self.hotzoneGraph.addWidget(self.graph_hz)
        self.graph_hz_ax = self.figure_hz.add_subplot()
        self.graph_hz_real_line, = self.graph_hz_ax.plot([0.0], [self.hz_function[1][0]], '.', color='navy')
        self.graph_hz_minline, = self.graph_hz_ax.plot([0, 0], [0, 0], 'r')
        self.graph_hz_line, = self.graph_hz_ax.plot([0, 0], [0, 0], color='tab:blue')
        self.graph_hz_ax.set_xlabel("Pulled Dist. (mm)")
        self.graph_hz_ax.set_ylabel("Hotzone Size (mm)")
        self.graph_hz_ax.grid(True)
        self.graph_hz.draw()
        
        self.figure_anglemini = plt.figure()
        self.graph_anglemini = FigureCanvas(self.figure_anglemini)
        self.angleMiniGraph.addWidget(self.graph_anglemini)
        self.graph_anglemini_ax = self.figure_anglemini.add_subplot()
        self.graph_anglemini_line, = self.graph_anglemini_ax.plot([0, 0], [0, 0])
        self.graph_anglemini_adialine, = self.graph_anglemini_ax.plot([0, 0], [0, 0], 'r')
        self.graph_anglemini_ax.set_xticks([])
        self.graph_anglemini_ax.set_yticks([])
        self.graph_anglemini.draw()
        
        self.figure_profmini = plt.figure()
        self.graph_profmini = FigureCanvas(self.figure_profmini)
        self.profMiniGraph.addWidget(self.graph_profmini)
        self.graph_profmini_ax = self.figure_profmini.add_subplot()
        self.graph_profmini_line, = self.graph_profmini_ax.plot([0, 0], [0, 0])
        self.graph_profmini_line_realish, = self.graph_profmini_ax.plot([0, 0], [0, 0], 'k--')
        self.graph_profmini_ax.set_xticks([])
        self.graph_profmini_ax.set_yticks([])
        self.graph_profmini.draw()
          
        # Timers
        self.mainLoop_timer = QTimer()
        self.mainLoop_timer.setInterval(self.main_to)
        self.mainLoop_timer.timeout.connect(self.mainLoop)
        self.mainLoop_timer.start()
        
        self.initLoop_timer = QTimer()
        self.initLoop_timer.setInterval(self.main_to)
        self.initLoop_timer.timeout.connect(self.initLoop)
        
        self.go2start_timer = QTimer()
        self.go2start_timer.setInterval(self.main_to)
        self.go2start_timer.timeout.connect(self.go2start_loop)
        
        self.pullLoop_timer = QTimer()
        self.pullLoop_timer.setInterval(self.main_to)
        self.pullLoop_timer.timeout.connect(self.pullLoop)
        
        self.calc_timer = QTimer()
        self.calc_timer.setInterval(self.main_to)
        self.calc_timer.timeout.connect(self.calcLoop)
        
        # DAQ connections
        self.update_daq_combos()
        self.srateSpin.valueChanged.connect(self.daq_init)
        self.daqChannelCombo.currentIndexChanged.connect(self.update_daq_combos)
        self.daqConfCombo.currentIndexChanged.connect(self.daq_init)
        self.daqmodeCombo.currentIndexChanged.connect(self.daq_init)
        self.daqvrangeSpin.valueChanged.connect(self.daq_init)
        self.responSpin.valueChanged.connect(self.daq_init)
        self.impedanceSpin.valueChanged.connect(self.daq_init)
        self.specsamplesSpin.valueChanged.connect(self.daq_init)
        
        # Motors connections
        self.resetmotorsBut.clicked.connect(self.init_motors)
        self.gotozeroBut.clicked.connect(self.go2start)
        self.startBut.clicked.connect(self.start_pulling)
        self.stopBut.clicked.connect(self.stop_pulling)
        self.tensionButton.clicked.connect(self.tension)
        self.cleaveBut.clicked.connect(self.cleave)
        self.loopBut.clicked.connect(self.loop)
        self.emerBut.clicked.connect(self.stop_all_motors)
        self.flameIOPosSpin.valueChanged.connect(self.fio_move2)
        self.brusherPosSpin.valueChanged.connect(self.fb_move2)
        self.leftPosSpin.valueChanged.connect(self.lp_move2)
        self.rightPosSpin.valueChanged.connect(self.rp_move2)
        self.enablemanualCheck.clicked.connect(self.toggle_manual_control)
        self.brusherMinSpanSpin.valueChanged.connect(self.update_minimum_hz)
        self.fSizeSpin.valueChanged.connect(self.update_minimum_hz)
        self.enhanceHZCheck.clicked.connect(self.enhanced_edge_warn)
        self.adaptivelCheck.clicked.connect(self.set_adaptive_vel)
        
        # Motor LEDs
        self.brInitLed_ef = self.make_led_clickable(self.brInitLed, self.core.motors.brusher)
        self.brHomeLed_ef = self.make_led_clickable(self.brHomeLed, self.core.motors.brusher)
        self.fioInitLed_ef = self.make_led_clickable(self.fioInitLed, self.core.motors.flame_io)
        self.fioHomeLed_ef = self.make_led_clickable(self.fioHomeLed, self.core.motors.flame_io)
        self.leftInitLed_ef = self.make_led_clickable(self.leftInitLed, self.core.motors.left_puller)
        self.leftHomeLed_ef = self.make_led_clickable(self.leftHomeLed, self.core.motors.left_puller)
        self.rightInitLed_ef = self.make_led_clickable(self.rightInitLed, self.core.motors.right_puller)
        self.rightHomeLed_ef = self.make_led_clickable(self.rightHomeLed, self.core.motors.right_puller)
        
        
        # Taper params connections
        self.distPriorRadio.clicked.connect(self.recalc_params)
        self.diamPriorRadio.clicked.connect(self.recalc_params)
        self.x0Spin.valueChanged.connect(self.recalc_params)
        self.l0Spin.valueChanged.connect(self.recalc_params)
        self.dwSpin.valueChanged.connect(self.recalc_params)
        self.alphaSpin.valueChanged.connect(self.recalc_params)
        self.loadHZBut.clicked.connect(self.load_hotzone)
        self.calcHZBut.clicked.connect(self.calc_hotzone)
        self.fiberdefsCheck.clicked.connect(self.toggle_fiber_defaults)
        self.d0Spin.valueChanged.connect(self.set_fiber_params)
        self.coredSpin.valueChanged.connect(self.set_fiber_params)
        self.cladSpin.valueChanged.connect(self.set_fiber_params)
        self.corenSpin.valueChanged.connect(self.set_fiber_params)
        self.wlSpin.valueChanged.connect(self.set_fiber_params)
        self.mednSpin.valueChanged.connect(self.set_fiber_params)
        
        # Misc. connections
        self.set0powBut.clicked.connect(self.set_ref_power)
        self.cmapCombo.currentTextChanged.connect(self.set_cmap)
        self.cmapRevCheck.clicked.connect(self.set_cmap)
        
         # Menu connections
        self.actionSave_Data.triggered.connect(self.action_save_data)
        self.actionClear_autosaved_data.triggered.connect(self.clear_autosaved_data)
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
        self.devsTabs.setCurrentIndex(0)
        self.designTabs.setCurrentIndex(0)
        
        self.reset_pull_stats()
        self.recalc_params()
        self.set_fiber_params()
        self.data.start_monitor()
        
    def set_daq_params(self):
        daq_dev_chan = self.daqChannelCombo.currentText()
        
        self.data.daq.sampling_rate = self.srateSpin.value()
        self.data.daq.device_channel = daq_dev_chan
        self.data.daq.term_config = self.daqConfCombo.currentText()
        self.data.daq.mode = self.daqmodeCombo.currentText()
        self.data.spectrum_points = self.specsamplesSpin.value()
        self.data.responsivity = self.responSpin.value()
        self.data.impedance = self.impedanceSpin.value()
        self.data.set_buffer(2*self.specsamplesSpin.value())
        
        if daq_dev_chan != "Simulation":
            volt_ranges = self.data.daq.get_volt_ranges(self.data.daq.get_dev_from_name(daq_dev_chan))
            closest_v_i = np.abs(np.array(volt_ranges) - self.daqvrangeSpin.value()).argmin()
            self.data.daq.scale = volt_ranges[closest_v_i]
        else:
            self.data.daq.scale = self.daqvrangeSpin.value()
    
    def set_motors_params(self):
        # Brusher
        self.core.motors.brusher.serial = self.brusherSerialText.text()
        self.core.motors.brusher.set_velocity(self.brusherVelSpin.value())
        self.core.motors.brusher.set_acceleration(self.brusherAccelSpin.value())
        self.core.brusher_x0 = self.brusherInitPosSpin.value()
        self.core.motors.brusher.min_span = self.brusherMinSpanSpin.value()
        if self.revdirCheck.isChecked():
            self.core.brusher_dir0 = -1
        else:
            self.core.brusher_dir0 = 1
        self.core.brusher_enhance_edge = self.enhanceHZCheck.isChecked()
            
        # In/Out
        self.core.motors.flame_io.serial = self.flameIOSerialText.text()
        self.core.motors.flame_io.set_velocity(self.flameIOVelSpin.value())
        self.core.motors.flame_io.set_acceleration(self.flameIOAccelSpin.value())
        self.core.flame_io_x0 = self.flameIOMovSpin.value()
        self.core.flame_io_hold = self.flameIOHoldSpin.value()
        self.core.flame_io_moveback = self.fioMovebackCheck.isChecked()
        self.core.flame_io_mb_start = self.flameIOTrigger1Spin.value()
        self.core.flame_io_mb_end = self.flameIOTrigger2Spin.value()
        self.core.flame_io_mb_to = self.flameIOMov2Spin.value()
        
        # Pullers
        self.core.motors.left_puller.serial = self.pullerLeftSerialText.text()
        self.core.motors.right_puller.serial = self.pullerRightSerialText.text()
        self.core.motors.left_puller.set_acceleration(self.pullerAccelSpin.value())
        self.core.motors.right_puller.set_acceleration(self.pullerAccelSpin.value())
        self.core.motors.left_puller.set_velocity(self.pullerVelSpin.value())
        self.core.motors.right_puller.set_velocity(self.pullerVelSpin.value())
        self.core.motors.left_puller.vel = self.pullerVelSpin.value()
        self.core.motors.right_puller.vel = self.pullerVelSpin.value()
        self.core.motors.left_puller.pull_vel = self.pullerPullVelSpin.value()
        self.core.motors.right_puller.pull_vel = self.pullerPullVelSpin.value()
        self.core.left_puller_x0 = self.pullerInitPosSpin.value()
        self.core.right_puller_x0 = self.pullerInitPosSpin.value()
        self.core.pullers_adaptive_vel = self.adaptivelCheck.isChecked()
        
    def set_fiber_params(self):
        n_cl = self.cladSpin.value()
        n_co = self.corenSpin.value()
        n_ratio = n_co/n_cl
        self.shape.set_parameters(self.wlSpin.value(), 1e-3*self.d0Spin.value()/2.0, 1e-3*self.coredSpin.value()/2.0, 
                                  n_ratio, self.mednSpin.value(), self.optimpointsSpin.value())
        self.shape.n_cladding = n_cl
        self.shape.n_core = n_co
        self.shape.n_core_ratio = self.shape.n_core/self.shape.n_cladding
        
    def toggle_fiber_defaults(self):
        self.d0Spin.setReadOnly(self.fiberdefsCheck.isChecked())
        self.coredSpin.setReadOnly(self.fiberdefsCheck.isChecked())
        self.corenSpin.setReadOnly(self.fiberdefsCheck.isChecked())
        self.cladSpin.setReadOnly(self.fiberdefsCheck.isChecked())
        self.wlSpin.setReadOnly(self.fiberdefsCheck.isChecked())
        self.mednSpin.setReadOnly(self.fiberdefsCheck.isChecked())
        
        if self.fiberdefsCheck.isChecked():
            buttons = QSpinBox.ButtonSymbols.NoButtons
            self.d0Spin.setValue(125.0)
            self.coredSpin.setValue(8.2)
            self.corenSpin.setValue(1.4492)
            self.cladSpin.setValue(1.444)
            self.wlSpin.setValue(1.55)
            self.mednSpin.setValue(1.0)
        else:
            buttons = QSpinBox.ButtonSymbols.UpDownArrows

        self.d0Spin.setButtonSymbols(buttons)
        self.coredSpin.setButtonSymbols(buttons)
        self.corenSpin.setButtonSymbols(buttons)
        self.cladSpin.setButtonSymbols(buttons)
        self.wlSpin.setButtonSymbols(buttons)
        self.mednSpin.setButtonSymbols(buttons)

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
            
            with open(filename, 'w') as f:
                json.dump(settings_dict, f, 
                    indent=4,
                    sort_keys=True,
                    separators=(',', ': ')
                )
    
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
    
    def load_default_settings(self):
        self.load_settings(self.default_settings_file)
        
    def save_data(self, file_prefix: str=""):
        parameters_file = file_prefix + "_parameters.json"
        
        parameters_dict = {}
        
        parameters_dict["Motors"] = {}
        parameters_dict["Motors"]["Brusher"] = {}
        parameters_dict["Motors"]["Brusher"]["Velocity"] = self.brusherVelSpin.value()
        parameters_dict["Motors"]["Brusher"]["Acceleration"] = self.brusherAccelSpin.value()
        parameters_dict["Motors"]["Brusher"]["Initial_pos"] = self.brusherInitPosSpin.value()
        parameters_dict["Motors"]["Brusher"]["Min_span"] = self.brusherMinSpanSpin.value()
        parameters_dict["Motors"]["Brusher"]["Flame_size"] = self.fSizeSpin.value()
        parameters_dict["Motors"]["Brusher"]["Edge_precision"] = self.enhanceHZCheck.isChecked()
        parameters_dict["Motors"]["Brusher"]["Reverse_dir"] = self.revdirCheck.isChecked()
        parameters_dict["Motors"]["FlameIO"] = {}
        parameters_dict["Motors"]["FlameIO"]["Velocity"] = self.flameIOVelSpin.value()
        parameters_dict["Motors"]["FlameIO"]["Acceleration"] = self.flameIOAccelSpin.value()
        parameters_dict["Motors"]["FlameIO"]["Initial_pos"] = self.flameIOMovSpin.value()
        parameters_dict["Motors"]["FlameIO"]["Hold_time"] = self.flameIOHoldSpin.value()
        parameters_dict["Motors"]["FlameIO"]["Change_prox"] = self.fioMovebackCheck.isChecked()
        parameters_dict["Motors"]["FlameIO"]["End_pos"] = self.flameIOMov2Spin.value()
        parameters_dict["Motors"]["FlameIO"]["Change_prox_start"] = self.flameIOTrigger1Spin.value()
        parameters_dict["Motors"]["FlameIO"]["Change_prox_end"] = self.flameIOTrigger2Spin.value()
        parameters_dict["Motors"]["Pullers"] = {}
        parameters_dict["Motors"]["Pullers"]["Max_velocity"] = self.pullerVelSpin.value()
        parameters_dict["Motors"]["Pullers"]["Pull_velocity"] = self.pullerPullVelSpin.value()
        parameters_dict["Motors"]["Pullers"]["Acceleration"] = self.pullerAccelSpin.value()
        parameters_dict["Motors"]["Pullers"]["Initial_pos"] = self.pullerInitPosSpin.value()
        parameters_dict["Motors"]["Pullers"]["Adjust_pull_vel"] = self.adaptivelCheck.isChecked()
        
        parameters_dict["DAQ"] = {}
        parameters_dict["DAQ"]["Dev/channel"] = self.daqChannelCombo.currentText()
        parameters_dict["DAQ"]["Term_config"] = self.daqConfCombo.currentText()
        parameters_dict["DAQ"]["Mode"] = self.daqmodeCombo.currentText()
        parameters_dict["DAQ"]["Sampling_rate"] = self.srateSpin.value()
        parameters_dict["DAQ"]["V_range"] = self.daqvrangeSpin.value()
        parameters_dict["DAQ"]["Spectrogram_samples"] = self.specsamplesSpin.value()
        parameters_dict["DAQ"]["Impedance"] = self.impedanceSpin.value()
        parameters_dict["DAQ"]["Responsivity"] = self.responSpin.value()
        
        parameters_dict["Profile"] = {}
        parameters_dict["Profile"]["Mode"] = self.profile_mode
        parameters_dict["Profile"]["Initial_diameter"] = self.d0Spin.value()
        if self.profile_mode == "Standard":
            parameters_dict["Profile"]["Waist_diameter"] = self.dwSpin.value()
            parameters_dict["Profile"]["Waist_length"] = self.setWaistLengthSpin.value()
            parameters_dict["Profile"]["Initial_HZ"] = self.l0Spin.value()
            parameters_dict["Profile"]["Dist_to_pull"] = self.x0Spin.value()
            parameters_dict["Profile"]["Alpha"] = self.alphaSpin.value()
        elif self.profile_mode == "Optimized":
            parameters_dict["Profile"]["Waist_diameter"] = self.dwoptSpin.value()
            parameters_dict["Profile"]["Waist_length"] = self.wlenoptSpin.value()
            parameters_dict["Profile"]["Initial_HZ"] = self.hz0optIndSpin.value()
            parameters_dict["Profile"]["Dist_to_pull"] = self.pulledoptIndSpin.value()
            parameters_dict["Profile"]["Calc_params"] = {}
            parameters_dict["Profile"]["Calc_params"]["Total_diameter"] = self.d0Spin.value()
            parameters_dict["Profile"]["Calc_params"]["Core_diameter"] = self.coredSpin.value()
            parameters_dict["Profile"]["Calc_params"]["Clad_n"] = self.cladSpin.value()
            parameters_dict["Profile"]["Calc_params"]["Core_n"] = self.corenSpin.value()
            parameters_dict["Profile"]["Calc_params"]["Medium_n"] = self.mednSpin.value()
            parameters_dict["Profile"]["Calc_params"]["Wavelength"] = self.wlSpin.value()
        parameters_dict["Profile"]["Transition_length"] = self.setTransLengthSpin.value()
        
        parameters_dict["Result"] = {}
        parameters_dict["Result"]["Total_pulled"] = self.totalPulledIndSpin.value()
        parameters_dict["Result"]["Est_waist_diam"] = self.waistDiamIndSpin.value()
        parameters_dict["Result"]["Est_waist_length"] = self.waistLengthIndSpin.value()
        parameters_dict["Result"]["Est_trans_length"] = self.transLengthIndSpin.value()
        parameters_dict["Result"]["Final_loss"] = self.transm_array[self.transm_i-1][1]
        
        with open(parameters_file, 'w') as f:
            json.dump(parameters_dict, f, 
                indent=4,
                sort_keys=True,
                separators=(',', ': ')
            )
            
        transm_file = file_prefix + "_transmission.txt"
        if self.transm_i > 0:
            np.savetxt(transm_file, self.transm_array[:self.transm_i-1])
            
        profile_file = file_prefix + "_profile.txt"
        np.savetxt(profile_file, np.array(self.profile).T)
        
        realish_profile_file = file_prefix + "_realish_profile.txt"
        np.savetxt(realish_profile_file, np.array(self.realish_profile).T)
        
        hz_file = file_prefix + "_hotzone.txt"
        np.savetxt(hz_file, np.array(self.hz_function).T)
        
        realhz_file = file_prefix + "_real_hotzone.txt"
        np.savetxt(realhz_file, np.array([self.rhz_x_array, self.rhz_array]).T)
        
        spec_file = file_prefix + "_spectrogram.txt"
        np.savetxt(spec_file, np.array(self.spectrogram_data).T)
    
    def enable_controls(self):
        widgets = self.findChildren(QWidget)
        for i in range(len(widgets)):
            widgets[i].setEnabled(True)
        self.stopBut.setEnabled(False)
        self.toggle_manual_control()
        self.toggle_fiber_defaults()
        
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
        self.cmapCombo.setEnabled(True)
        self.cmapRevCheck.setEnabled(True)
        self.stopBut.setEnabled(True)
        
    def reset_pull_stats(self):
        self.core.reset_pull_stats()
        self.set_motors_params()
        self.timeleftLabel.setText(f"Time left: {self.core.get_time_left(self.total_to_pull, 0.0):.2f} s")
        self.timeleftBar.setValue(0)
        self.totalPulledIndSpin.setValue(0.0)
        self.waistDiamIndSpin.setValue(self.d0Spin.value())
        self.waistLengthIndSpin.setValue(0.0)
        self.transLengthIndSpin.setValue(0.0)
        self.hotzoneIndSpin.setValue(0.0)
        self.rhz_array = []
        self.rhz_x_array = []
        self.transm_array = np.zeros((60000, 2))
        self.transm_i = 0
        self.update_graph([[0.0], [self.hz_function[1][0]]], self.graph_hz_real_line, self.graph_hz_ax, self.graph_hz)
        self.hz_sweep_n = 0

    # Timer functions
    def mainLoop(self):
        if not self.busy:
            self.busy = True
                
            # Update power monitor
            if self.monpowCheck.isChecked():
                self.transpowIndSpin.setValue(self.data.get_monitor_power(db=True))
                self.lossIndSpin.setValue(self.refpowIndSpin.value() - self.transpowIndSpin.value())
                
            # Update motor positions
            br_pos = self.core.motors.brusher.get_position()
            fio_pos = self.core.motors.flame_io.get_position()
            l_pos = self.core.motors.left_puller.get_position()
            r_pos = self.core.motors.right_puller.get_position()
            self.brusherPosIndSpin.setValue(br_pos)
            self.flameIOPosIndSpin.setValue(fio_pos)
            self.leftPosIndSpin.setValue(l_pos)
            self.rightPosIndSpin.setValue(r_pos)
            
            # Update indicators
            self.brusherIndSlider.setValue(int(1000.0*br_pos/self.core.motors.brusher.max_pos))
            self.inoutIndSlider.setValue(int(1000.0*fio_pos/self.core.motors.flame_io.max_pos))
            self.pullLeftIndSlider.setValue(int(1000.0*l_pos/self.core.motors.left_puller.max_pos))
            self.pullRightIndSlider.setValue(int(1000.0*r_pos/self.core.motors.right_puller.max_pos))
            
            # Update leds
            if self.core.motors.brusher.ok and self.core.motors.brusher.movement == self.core.motors.brusher.MoveDirection.STOPPED:
                self.brInitLed.setPixmap(QPixmap(f"{respath}/green_led.png"))
            elif self.core.motors.brusher.ok and self.core.motors.brusher.movement != self.core.motors.brusher.MoveDirection.STOPPED:
                self.brInitLed.setPixmap(QPixmap(f"{respath}/yellow_led.png"))
            elif self.core.motors.brusher.error:
                self.brInitLed.setPixmap(QPixmap(f"{respath}/red_led.png"))
            
            if self.core.motors.flame_io.ok and self.core.motors.flame_io.movement == self.core.motors.flame_io.MoveDirection.STOPPED:
                self.fioInitLed.setPixmap(QPixmap(f"{respath}/green_led.png"))
            elif self.core.motors.flame_io.ok and self.core.motors.flame_io.movement != self.core.motors.flame_io.MoveDirection.STOPPED:
                self.fioInitLed.setPixmap(QPixmap(f"{respath}/yellow_led.png"))
            elif self.core.motors.flame_io.error:
                self.fioInitLed.setPixmap(QPixmap(f"{respath}/red_led.png"))
            
            if self.core.motors.left_puller.ok and self.core.motors.left_puller.movement == self.core.motors.left_puller.MoveDirection.STOPPED:
                self.leftInitLed.setPixmap(QPixmap(f"{respath}/green_led.png"))
            elif self.core.motors.left_puller.ok and self.core.motors.left_puller.movement != self.core.motors.left_puller.MoveDirection.STOPPED:
                self.leftInitLed.setPixmap(QPixmap(f"{respath}/yellow_led.png"))
            elif self.core.motors.left_puller.error:
                self.leftInitLed.setPixmap(QPixmap(f"{respath}/red_led.png"))
            
            if self.core.motors.right_puller.ok and self.core.motors.right_puller.movement == self.core.motors.right_puller.MoveDirection.STOPPED:
                self.rightInitLed.setPixmap(QPixmap(f"{respath}/green_led.png"))
            elif self.core.motors.right_puller.ok and self.core.motors.right_puller.movement != self.core.motors.right_puller.MoveDirection.STOPPED:
                self.rightInitLed.setPixmap(QPixmap(f"{respath}/yellow_led.png"))
            elif self.core.motors.right_puller.error:
                self.rightInitLed.setPixmap(QPixmap(f"{respath}/red_led.png"))
            
            self.busy = False
    
    def initLoop(self):
        if not self.init_busy:
            self.init_busy = True
            
            connected = [False]*4
            homed = [False]*4
            error = [False]*4
            
            # Check connections
            if self.core.motors.brusher.ok:
                connected[0] = True
            if self.core.motors.flame_io.ok:
                connected[1] = True
            if self.core.motors.left_puller.ok:
                connected[2] = True
            if self.core.motors.right_puller.ok:
                connected[3] = True
            
            # Check errors
            if self.core.motors.brusher.error:
                error[0] = True
            if self.core.motors.flame_io.error:
                error[1] = True
            if self.core.motors.left_puller.error:
                error[2] = True
            if self.core.motors.right_puller.error:
                error[3] = True
            
            # Check if pullers are ok and initialize flame motors
            if connected[2] and connected[3]:
                if self.core.motors.left_puller.danger_zone():
                    self.core.motors.left_puller.go_to(self.core.motors.left_puller.safe_range[1])
                if self.core.motors.right_puller.danger_zone():
                    self.core.motors.right_puller.go_to(self.core.motors.right_puller.safe_range[1])
                
                if not connected[0] and not self.core.motors.brusher.initing:
                    self.brInitLed.setPixmap(QPixmap(f"{respath}/yellow_led.png"))
                    self.core.init_brusher_as_default(True, self.brSimCheck.isChecked())
                if not connected[1] and not self.core.motors.flame_io.initing:
                    self.fioInitLed.setPixmap(QPixmap(f"{respath}/yellow_led.png"))
                    self.core.init_flameio_as_default(True, self.fioSimCheck.isChecked())
            
            if connected[0]:
                if self.core.motors.brusher.homed:
                    self.brHomeLed.setPixmap(QPixmap(f"{respath}/green_led.png"))
                    homed[0] = True
                elif self.core.motors.brusher.homing:
                    self.brHomeLed.setPixmap(QPixmap(f"{respath}/yellow_led.png"))
                else:
                    self.core.motors.brusher.home()
            if connected[1]:
                if self.core.motors.flame_io.homed:
                    self.fioHomeLed.setPixmap(QPixmap(f"{respath}/green_led.png"))
                    homed[1] = True
                elif self.core.motors.flame_io.homing:
                    self.fioHomeLed.setPixmap(QPixmap(f"{respath}/yellow_led.png"))
                else:
                    self.core.motors.flame_io.home()
            if connected[2]:
                if self.core.motors.left_puller.homed:
                    self.leftHomeLed.setPixmap(QPixmap(f"{respath}/green_led.png"))
                    homed[2] = True
                elif self.core.motors.left_puller.homing:
                    self.leftHomeLed.setPixmap(QPixmap(f"{respath}/yellow_led.png"))
                else:
                    self.core.motors.left_puller.home()
            if connected[3]:
                if self.core.motors.right_puller.homed:
                    self.rightHomeLed.setPixmap(QPixmap(f"{respath}/green_led.png"))
                    homed[3] = True
                elif self.core.motors.right_puller.homing:
                    self.rightHomeLed.setPixmap(QPixmap(f"{respath}/yellow_led.png"))
                else:
                    self.core.motors.right_puller.home()
            
            self.init_busy = False
            
            if all(connected + homed):
                self.core.standby = True
                self.core.running_process = True
                self.initLoop_timer.stop()
                self.initLoop_active = False
                self.init_motors_i = 0
                self.resetmotorsBut.setEnabled(True)
            elif all([a | b for a, b in zip(connected[2:], error[2:])]) and not all(connected[2:]):
                self.initLoop_timer.stop()
                self.initLoop_active = False
                if self.init_motors_i < self.init_motors_tries:
                    self.init_motors_i += 1
                    self.init_motors()
            elif all([a | b for a, b in zip(connected, error)]) and not all(connected):
                self.initLoop_timer.stop()
                self.initLoop_active = False
                if self.init_motors_i < self.init_motors_tries:
                    self.init_motors_i += 1
                    self.init_motors()
                else:
                    self.core.standby = True
                    self.core.running_process = True
                    self.init_motors_i = 0
                    self.resetmotorsBut.setEnabled(True)            
    
    def pullLoop(self):
        if not self.pull_busy:
            self.pull_busy = True
            
            # Update time
            time_left = self.core.get_time_left()
            total_time = self.core.get_time_left(total_pulled=0.0)
            self.timeleftLabel.setText(f"Time left: {time_left:.2f} s")
            self.timeleftBar.setValue(int(1000.0*(1 - time_left/total_time)))
            
            tp = self.core.total_pulled
            
            # Update real hotzone
            curr_hz_sweep_n = len(self.core.rhz_edges)
            if len(self.core.rhz_edges) > self.hz_sweep_n:
                if curr_hz_sweep_n > 1:
                    rhz = np.abs(self.core.rhz_edges[-1] - self.core.rhz_edges[-2])
                    self.hotzoneIndSpin.setValue(rhz)
                    if self.core.pulling:
                        self.rhz_array.append(rhz)
                        self.rhz_x_array.append(tp)
                        rhz_data = np.array([self.rhz_x_array, self.rhz_array])
                        self.update_graph(rhz_data, self.graph_hz_real_line, self.graph_hz_ax, self.graph_hz)
                        
                        # Update profile graph with profile from real hz
                        self.realish_profile = self.shape.profile_from_hz(np.array(self.rhz_x_array), np.array(self.rhz_array))                        
                        self.update_graph(self.realish_profile, self.graph_profmini_line_realish, self.graph_profmini_ax, self.graph_profmini)
                        
                        curr_d = 2000*self.realish_profile[1][-1]
                        self.waistDiamIndSpin.setValue(curr_d)
                else:
                    rhz = self.hz_function[1][0]
                    self.hotzoneIndSpin.setValue(rhz)
                    if self.core.pulling:
                        self.rhz_array.append(rhz)
                        self.rhz_x_array.append(0.0)
                        rhz_data = np.array([self.rhz_x_array, self.rhz_array])
                        self.update_graph(rhz_data, self.graph_hz_real_line, self.graph_hz_ax, self.graph_hz)
                self.hz_sweep_n += 1
                
            # Update est. values
            # Designed hz         
            # curr_hz = self.hz_function[1][np.abs(self.hz_function[0] - tp).argmin()]
            
            # Real hz
            if len(self.rhz_array) > 0:
                curr_hz = self.rhz_array[-1]
            else:
                curr_hz = 0
            
            self.totalPulledIndSpin.setValue(tp)
            self.waistLengthIndSpin.setValue(curr_hz)
            self.transLengthIndSpin.setValue((tp + self.hz_function[1][0] - curr_hz)/2)
                
            # Update transmission
            if self.core.pulling and self.core.motors.left_puller.moving and self.core.motors.right_puller.moving:
                self.transm_array[self.transm_i][0] = tp
                self.transm_array[self.transm_i][1] = self.data.get_monitor_power(db=False)
                self.transm_i += 1
                tdata = self.transm_array[:self.transm_i]
                if self.transm_i > self.max_tpts:
                    idxs = np.linspace(1, self.transm_i - 2, self.max_tpts - 2, dtype=int)
                    tdata = np.concatenate(([tdata[0]], tdata[idxs], [tdata[-1]]))
                    
                self.update_graph(tdata.T, self.graph_pow_line, self.graph_pow_ax, self.graph_pow)
                
            # Update Spectrogram
            if self.core.motors.left_puller.moving and self.core.motors.right_puller.moving:
                self.data.spectrogram_pause = False
            else:
                self.data.spectrogram_pause = True
            if self.core.pulling and not self.data.spectrogram_pause:
                if not self.data.spectrogram_running:
                    self.data.cutoff_f = 2*self.pullerPullVelSpin.value()*1000
                    self.data.start_spectrogram(0.1, True, 0.15)
                
                if len(self.data.spectra) > 0:
                    tp_arr = np.linspace(0.0, tp, len(self.data.spectra))
                    if len(self.data.spectra) > self.max_spectra:
                        data = np.array(self.data.spectra[-self.max_spectra:]).T
                        tp0 = tp_arr[-self.max_spectra]
                    else:
                        data = np.array(self.data.spectra).T
                        tp0 = 0.0
                    freqs = self.data.spectra_freqs
                    if data.shape[1] > self.graph_spec.width():
                        data = cv2.resize(data, (self.graph_spec.width(), len(freqs)))
                    self.graph_spec_img.set_data(data)
                    self.graph_spec_img.set_extent((tp0, tp, 1e-3*freqs[0]/self.pullerPullVelSpin.value(), 1e-3*freqs[-1]/self.pullerPullVelSpin.value()))
                    self.graph_spec_img.set_clim(data.min(), data.max())
                    self.graph_spec.draw()
                    self.graph_spec.flush_events()
                    self.spectrogram_data = data
            
            # Check if ended
            if self.core.standby:
                self.pullLoop_timer.stop()
                self.stop_pulling(True)
                
            self.pull_busy = False
        
    # DAQ functions
    def daq_init(self):
        if not self.daq_busy:
            self.daq_busy = True
            self.set_daq_params()
            if self.daqChannelCombo.currentText() == "Simulation":
                self.data.init_daq_as_default(simulate=True)
            else:
                self.data.init_daq_as_default(simulate=False)
            self.daq_busy = False
    
    def update_daq_combos(self):
        if not self.daq_busy:
            self.daq_busy = True
            self.data.daq.get_devices()
            self.data.daq.get_ai_channels()
            
            daq_dev_chan = self.daqChannelCombo.currentText()
            
            prev_devch = self.daqChannelCombo.currentText()
            prev_term = self.daqConfCombo.currentText()
            
            self.daqChannelCombo.clear()
            self.daqChannelCombo.addItem("Simulation")
            for channel in self.data.daq.channels_names:
                self.daqChannelCombo.addItem(channel)
            
            if prev_devch in self.data.daq.channels_names:
                self.daqChannelCombo.setCurrentText(prev_devch)
            elif prev_devch == "Simulation":
                self.daqChannelCombo.setCurrentText("Simulation")
            else:
                self.daqChannelCombo.setCurrentText(self.data.daq.channels_names[0])
            
            if self.daqChannelCombo.currentText() != "Simulation":
                try:
                    chan_i = self.data.daq.channels_names.index(self.daqChannelCombo.currentText())
                except:
                    chan_i = -1
                
                if chan_i >= 0:
                    confs = self.data.daq.get_term_configs(self.data.daq.channels[chan_i])   
                    self.daqConfCombo.clear()
                    self.daqConfCombo.addItem("DEFAULT")
                    conf_names = []
                    for conf in confs:
                        conf_name = conf.name
                        conf_names.append(conf_name)
                        self.daqConfCombo.addItem(conf_name)
                    if prev_term in conf_names:
                        self.daqConfCombo.setCurrentText(prev_term)
                        
            if daq_dev_chan != "Simulation":
                volt_ranges = self.data.daq.get_volt_ranges(self.data.daq.get_dev_from_name(daq_dev_chan))
            else:
                volt_ranges = [0.01, 100]
            self.daqvrangeSpin.setMinimum(volt_ranges[0])
            self.daqvrangeSpin.setMaximum(volt_ranges[-1])
                
            self.daq_busy = False
            
            self.daq_init()
               
    def set_ref_power(self):
        self.refpowIndSpin.setValue(self.transpowIndSpin.value())
        
    # Motors functions
    def toggle_manual_control(self):
        if self.enablemanualCheck.isChecked():
            self.flameIOPosSpin.setValue(self.flameIOPosIndSpin.value())
            self.brusherPosSpin.setValue(self.brusherPosIndSpin.value())
            self.leftPosSpin.setValue(self.leftPosIndSpin.value())
            self.rightPosSpin.setValue(self.rightPosIndSpin.value())
        
        self.flameIOPosSpin.setEnabled(self.enablemanualCheck.isChecked())
        self.brusherPosSpin.setEnabled(self.enablemanualCheck.isChecked())
        self.leftPosSpin.setEnabled(self.enablemanualCheck.isChecked())
        self.rightPosSpin.setEnabled(self.enablemanualCheck.isChecked())
    
    def fb_move2(self):
        if self.enablemanualCheck.isChecked():
            self.core.motors.brusher.go_to(self.brusherPosSpin.value())
        
    def fio_move2(self):
        if self.enablemanualCheck.isChecked():
            self.core.motors.flame_io.go_to(self.flameIOPosSpin.value())
        
    def lp_move2(self):
        if self.enablemanualCheck.isChecked():
            self.core.motors.left_puller.go_to(self.leftPosSpin.value())
        
    def rp_move2(self):
        if self.enablemanualCheck.isChecked():
            self.core.motors.right_puller.go_to(self.rightPosSpin.value())
        
    def stop_all_motors(self):
        self.stop_pulling(save_data=False)
        self.core.motors.brusher.stop()
        self.core.motors.flame_io.stop()
        self.core.motors.left_puller.stop()
        self.core.motors.right_puller.stop()
        self.core.motors.flame_io.go_to(0.0)
        
        self.going_2start = False
        self.go2start_timer.stop()
        
        self.initLoop_timer.stop()
        self.initLoop_active = False
        self.init_motors_i = 0
        self.resetmotorsBut.setEnabled(True)
        
    def init_motors(self):
        if not self.initLoop_active:
            self.set_motors_params()
            
            self.resetmotorsBut.setEnabled(False)
            self.initLoop_active = True
            self.init_busy = False
            self.initLoop_timer.start()
            
            if not self.core.motors.left_puller.ok:
                self.leftInitLed.setPixmap(QPixmap(f"{respath}/yellow_led.png"))
                self.core.init_puller_l_as_default(True, self.leftSimCheck.isChecked())
            if not self.core.motors.right_puller.ok:
                self.rightInitLed.setPixmap(QPixmap(f"{respath}/yellow_led.png"))
                self.core.init_puller_r_as_default(True, self.rightSimCheck.isChecked())
        
    def go2start(self):
        if not self.going_2start:
            self.going_2start = True
            self.go2start_timer.start()
        
    def go2start_loop(self):
        if not self.go2start_busy:
            self.go2start_busy = True
            
            bOk = np.abs(self.core.brusher_pos - self.brusherInitPosSpin.value()) <= self.core.pos_check_precision
            fOk = np.abs(self.core.flame_io_pos - 0.0) <= self.core.pos_check_precision
            lOk = np.abs(self.core.puller_left_pos - self.pullerInitPosSpin.value()) <= self.core.pos_check_precision
            rOk = np.abs(self.core.puller_right_pos - self.pullerInitPosSpin.value()) <= self.core.pos_check_precision
            
            if not bOk and not self.core.motors.brusher.moving:
                self.core.motors.brusher.go_to(self.brusherInitPosSpin.value())
            if not fOk and not self.core.motors.flame_io.moving:
                self.core.motors.flame_io.go_to(0.0)
                
            if bOk and fOk:
                if not self.core.motors.left_puller.moving:
                    self.core.motors.left_puller.go_to(self.pullerInitPosSpin.value())
                if not self.core.motors.right_puller.moving:
                    self.core.motors.right_puller.go_to(self.pullerInitPosSpin.value())
                
            if bOk and fOk and lOk and rOk:
                self.going_2start = False
                self.go2start_timer.stop()
            
            self.go2start_busy = False
        
    def start_pulling(self):
        self.disable_controls()
        self.core.force_hz_edge = self.edgeStopCheck.isChecked()
        self.core.brusher_enhance_edge = self.enhanceHZCheck.isChecked()
        self.core.pullers_adaptive_vel = self.adaptivelCheck.isChecked()
        self.core.flame_size = self.fSizeSpin.value()
        
        self.set_motors_params()
        self.daq_init() 
        
        self.reset_pull_stats()
        
        hz_function = self.hz_function
        if self.manualStopCheck.isChecked():
            hz_function[0][-1] = 200.0
        self.core.start_process(hz_function)
        self.pullLoop_timer.start()
        
    def stop_pulling(self, save_data=False):
        self.pullLoop_timer.stop()
        self.core.stop_pulling()
        self.data.stop_spectrogram()
        self.enable_controls()
        self.core.motors.left_puller.set_velocity(self.pullerVelSpin.value())
        self.core.motors.right_puller.set_velocity(self.pullerVelSpin.value())
        self.timeleftBar.setValue(0)
        self.timeleftLabel.setText(f"Time left: 0.0 s")
        
        # Save data just to be safe
        if save_data:
            data_bckp_dir = f"{rootpath}/data_bckp/{datetime.now().strftime("%Y_%m_%d")}"
            Path(data_bckp_dir).mkdir(parents=True, exist_ok=True)
            self.save_data(f"{data_bckp_dir}/auto_{datetime.now().strftime("%H_%M_%S")}")
    
    def tension(self):
        if self.core.standby:
            text = "The tension process will now start.\n" + \
                    f"The puller motors will move 0.1 mm back (each), to tension the fiber. Repeat as needed.\n\n" + \
                    "Click 'Ok' to continue."
            box_resp = QMessageBox.information(self, "Tension taper", text, QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel)
            if box_resp == QMessageBox.StandardButton.Ok:
                self.core.motors.left_puller.move_relative(-0.1)
                self.core.motors.right_puller.move_relative(-0.1)
            
    def cleave(self):
        if self.core.standby:
            text = "Caution: this procedure is a gamble, and results are not guaranteed.\n" + \
                "There two options available:\n\n" + \
                "1. The pullers move back 2 mm each at the normal pulling velocity, slowly stretching " + \
                "the fiber until it breaks. This procedure is best when used on a looped taper.\n\n" + \
                "2. The puller motors will move 5 mm back each at 0.1 m/s, accelerating with 0.9g.\n\n" + \
                "Both methods should, in theory, break the taper as smoothly as possible right in the middle.\n" + \
                "If you are able, a very tiny scratch can help achieve good a result.\n\n" + \
                "Which method do you want to try?"
                
            cleave_box = QMessageBox()
            cleave_box.setWindowTitle("About to attempt cleaving")
            cleave_box.setIcon(QMessageBox.Icon.Warning)
            cleave_box.setText(text)
            cleave_box.addButton("Smooth cleave", QMessageBox.ButtonRole.YesRole)
            cleave_box.addButton("Brute cleave", QMessageBox.ButtonRole.NoRole)
            cleave_box.addButton(QMessageBox.StandardButton.Cancel)
            choose_slow = cleave_box.exec()

            # if choose_slow == QMessageBox.ButtonRole.YesRole:  # Doesn't work
            # if choose_slow == QMessageBox.StandardButton.Yes:  # Doesn't work
            if choose_slow == 2:  # Why 2? That's what's returned, but why? How can we make this safer to future changes?
                self.core.cleave_mode = "slow"
                self.core.cleave_dist = 2.0
                self.core.cleaving = True
            elif choose_slow == 3:
                self.core.cleave_mode = "fast"
                self.core.cleave_dist = 5.0
                self.core.cleaving = True
        
    def loop(self):
        if self.core.standby:
            loop_mf = self.loopmfSpin.value()
            loop_mb = self.loopmbSpin.value()
            text = "The loop wizard will now start.\n" + \
                    f"The puller motors will move {loop_mf:.2f} mm forward (each), to loosen the fiber.\n\n" + \
                    "Click 'Ok' to continue."
            box_resp = QMessageBox.information(self, "Loop Wizard", text, QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel)
            
            if box_resp == QMessageBox.StandardButton.Ok:
                self.core.loop_dist_bw = loop_mb
                self.core.loop_dist_fw = loop_mf
                self.core.looping = True
                while not self.core.loop_loosened:
                    time.sleep(0.1)

                text = "Now twist as needed.\n\n" + \
                    "Click 'Ok' to continue when done."
                box_resp = QMessageBox.information(self, "Loop Wizard", text, QMessageBox.StandardButton.Ok)
                
                text = f"The puller motors will now move {loop_mb:.2f} mm back (each), to tension the loop" + \
                        " and control its diameter.\n\n" + \
                        "Click 'Ok' to continue."
                box_resp = QMessageBox.information(self, "Loop Wizard", text, QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel)
                
                if box_resp == QMessageBox.StandardButton.Ok:
                    self.core.loop_looped = True
                    while self.core.looping:
                        time.sleep(0.1)
                    
                    text = "Thats it! Your loop should be complete!\n\n" + \
                        "Click 'Ok' to end the wizard."
                    QMessageBox.information(self, "Loop Wizard", text, QMessageBox.StandardButton.Ok)
                else:
                    self.core.looping = False
                    self.core.loop_started = False
                    self.core.loop_loosened = False
                    self.core.loop_looped = False
                    self.core.loop_tensioning = False
                    self.core.loop_tensioned = False
                    
    
    # Taper params functions
    def recalc_params(self):
        alpha = self.alphaSpin.value()
        x0 = self.x0Spin.value()
        dw = 1e-3*self.dwSpin.value()
        d0 = 1e-3*self.d0Spin.value()
        hz0 = self.l0Spin.value()

        if self.distPriorRadio.isChecked():
            rw = self.shape.calc_rw_uniform_hz(hz0, alpha, x0, d0/2.0)
            self.dwSpin.setValue(1e3*2*rw)
            dw = 2*rw
        elif self.diamPriorRadio.isChecked():
            x0 = self.shape.calc_pull_uniform_hz(hz0, alpha, dw/2.0, d0/2.0)
            self.x0Spin.setValue(x0)
            
        z0 = (1 - alpha)*x0/2
        lw = hz0 + alpha*x0

        self.setTransLengthSpin.setValue(z0)
        self.setWaistLengthSpin.setValue(lw)
        self.set_motors_params()
        self.timeleftLabel.setText(f"Time left: {self.core.get_time_left(x0, 0.0):.2f} s")
        
        if np.abs(alpha) > 0.01:
            hz_data = self.shape.uniform_hz(hz0, alpha, x0)
        else:
            hz_data = np.array([np.linspace(0.0, x0, self.shape.n_points), np.ones(self.shape.n_points)*hz0])
        
        profile_data = self.shape.profile_from_hz(hz_data[0], hz_data[1])
        self.profile_angles = self.shape.profile_angles(profile_data[0], profile_data[1])
        ideal_prof = self.shape.ideal_adiabatic_profile(dw/2.0, 1.0)
        self.adiab_angles = self.shape.profile_angles(ideal_prof[0], ideal_prof[1])
        self.hz_function = hz_data
        self.profile = profile_data
        
        self.update_graph(hz_data, self.graph_hz_line, self.graph_hz_ax, self.graph_hz)
        self.update_graph([[0.0], [self.hz_function[1][0]]], self.graph_hz_real_line, self.graph_hz_ax, self.graph_hz)
        self.update_graph(profile_data, self.graph_profmini_line, self.graph_profmini_ax, self.graph_profmini)
        self.update_graph(np.array([[0, 0], [0, 0]]), self.graph_profmini_line_realish, self.graph_profmini_ax, self.graph_profmini)
        self.update_graph(np.array([ideal_prof[1], self.adiab_angles]), self.graph_anglemini_adialine, self.graph_anglemini_ax, self.graph_anglemini)
        self.update_graph(np.array([profile_data[1], self.profile_angles]), self.graph_anglemini_line, self.graph_anglemini_ax, self.graph_anglemini)
        self.update_minimum_hz()
        
        self.total_to_pull = x0
        
        self.profile_mode = "Standard"
        
    def update_graph(self, data, graph_line, graph_ax, graph_canvas, xlims=None, ylims=None):
        graph_line.set_xdata(data[0])
        graph_line.set_ydata(data[1])
        
        if xlims == None and ylims == None:
            graph_ax.relim()
            graph_ax.autoscale(True)
        else:
            if isinstance(xlims, list) and isinstance(ylims, list):
                graph_ax.set_xlim(xlims)
                graph_ax.set_ylim(ylims)
                graph_ax.autoscale(False)
            elif isinstance(xlims, list):
                graph_ax.relim()
                graph_ax.set_xlim(xlims)
                graph_ax.autoscale(True, axis='y')
            elif isinstance(ylims, list):
                graph_ax.relim()
                graph_ax.set_ylim(ylims)
                graph_ax.autoscale(True, axis='x')
        graph_canvas.draw()
        graph_canvas.flush_events()
        
    def set_cmap(self):
        cmap = self.cmapCombo.currentText()
        if self.cmapRevCheck.isChecked():
            cmap = f"{cmap}_r"
        self.graph_spec_img.set_cmap(cmap)
        self.graph_spec.draw()
        self.graph_spec.flush_events()
        
    def update_minimum_hz(self):
        self.core.motors.brusher.min_span = self.brusherMinSpanSpin.value()
        self.min_hz = self.fSizeSpin.value() + self.brusherMinSpanSpin.value()
        self.update_graph(np.array([[0.0, self.total_to_pull], [self.min_hz, self.min_hz]]), 
                          self.graph_hz_minline, self.graph_hz_ax, self.graph_hz)
        
    def load_hotzone(self):
        file = QFileDialog.getOpenFileName(self, "Load hotzone profile", self.last_dir, "Text files (*.txt *.csv *.dat)")
        filename = file[0]
        
        self.last_dir = os.path.dirname(os.path.realpath(filename)).replace("\\", "/")
       
        if filename != "":
            directory = os.path.dirname(os.path.realpath(filename)).replace("\\", "/")
            Path(directory).mkdir(parents=True, exist_ok=True)

            delimiters = [" ", ",", "\t", ";"]
            del_i = 0
            seems_ok = False
            while not seems_ok and del_i < len(delimiters):
                try:
                    loaded_data = np.loadtxt(filename, delimiter=delimiters[del_i]).T
                    if loaded_data.shape[0] == 2 and loaded_data.shape[1] > 1:
                        seems_ok = True
                        self.hz_function = np.array([loaded_data[0], loaded_data[1]])
                        self.total_to_pull = self.hz_function[0][-1]
                        
                        profile_data = self.shape.profile_from_hz(self.hz_function[0], self.hz_function[1])
                        self.profile_angles = self.shape.profile_angles(profile_data[0], profile_data[1])
                        ideal_prof = self.shape.ideal_adiabatic_profile(profile_data[1].min(), 1.0)
                        self.adiab_angles = self.shape.profile_angles(ideal_prof[0], ideal_prof[1])
                        self.profile = profile_data
                        
                        self.update_graph(self.hz_function, self.graph_hz_line, self.graph_hz_ax, self.graph_hz)
                        self.update_graph([[0.0], [self.hz_function[1][0]]], self.graph_hz_real_line, self.graph_hz_ax, self.graph_hz)
                        self.update_graph(profile_data, self.graph_profmini_line, self.graph_profmini_ax, self.graph_profmini)
                        self.update_graph(np.array([[0, 0], [0, 0]]), self.graph_profmini_line_realish, self.graph_profmini_ax, self.graph_profmini)
                        self.update_graph(np.array([ideal_prof[1], self.adiab_angles]), self.graph_anglemini_adialine, self.graph_anglemini_ax, self.graph_anglemini)
                        self.update_graph(np.array([profile_data[1], self.profile_angles]), self.graph_anglemini_line, self.graph_anglemini_ax, self.graph_anglemini)
                        self.update_minimum_hz()

                        self.dwoptSpin.setValue(2e3*profile_data[1].min())
                        self.wlenoptSpin.setValue(self.hz_function[1][-1])
                        self.setWaistLengthSpin.setValue(self.hz_function[1][-1])
                        self.pulledoptIndSpin.setValue(self.hz_function[0][-1])
                        self.hz0optIndSpin.setValue(self.hz_function[1][0])
                        self.setTransLengthSpin.setValue((self.hz_function[0][-1] + self.hz_function[1][0] - self.hz_function[1][-1])/2)
                        self.set_motors_params()
                        self.timeleftLabel.setText(f"Time left: {self.core.get_time_left(self.total_to_pull, 0.0):.2f} s")
                        
                        self.profile_mode = "Loaded"
                        self.statusBar.showMessage(f"Hotzone file loaded")
                except:
                    pass
                del_i += 1
            if not seems_ok:
                self.statusBar.showMessage(f"Hotzone file loading failed! The file format is probably wrong.")
    
    def calc_hotzone(self):
        self.statusBar.showMessage(f"Calculating ideal feasible profile. Please wait...")
        self.shape.real_adiabatic_profile_async(0.5e-3*self.dwoptSpin.value(), self.min_hz, self.ffactorSpin.value())
        
        self.calc_timer.start()
    
    def calcLoop(self):
        if self.shape.calc_finished:
            self.calc_timer.stop()
            
            try:
                z_arr = self.shape.calc_z_array
                r_arr = self.shape.calc_r_array
                x_arr, l_arr = self.shape.hz_from_profile(z_arr, r_arr, self.wlenoptSpin.value())
                
                self.hz_function = np.array([x_arr, l_arr])
                self.total_to_pull = self.hz_function[0][-1]
                
                profile_data = self.shape.profile_from_hz(self.hz_function[0], self.hz_function[1])
                self.profile_angles = self.shape.profile_angles(profile_data[0], profile_data[1])
                ideal_prof = self.shape.ideal_adiabatic_profile(profile_data[1].min(), 1.0)
                self.adiab_angles = self.shape.profile_angles(ideal_prof[0], ideal_prof[1])
                self.profile = profile_data
                
                self.update_graph(self.hz_function, self.graph_hz_line, self.graph_hz_ax, self.graph_hz)
                self.update_graph([[0.0], [self.hz_function[1][0]]], self.graph_hz_real_line, self.graph_hz_ax, self.graph_hz)
                self.update_graph(profile_data, self.graph_profmini_line, self.graph_profmini_ax, self.graph_profmini)
                self.update_graph(np.array([[0, 0], [0, 0]]), self.graph_profmini_line_realish, self.graph_profmini_ax, self.graph_profmini)
                self.update_graph(np.array([ideal_prof[1], self.adiab_angles]), self.graph_anglemini_adialine, self.graph_anglemini_ax, self.graph_anglemini)
                self.update_graph(np.array([profile_data[1], self.profile_angles]), self.graph_anglemini_line, self.graph_anglemini_ax, self.graph_anglemini)
                self.update_minimum_hz()

                self.dwoptSpin.setValue(2e3*profile_data[1].min())
                self.wlenoptSpin.setValue(self.hz_function[1][-1])
                self.setWaistLengthSpin.setValue(self.hz_function[1][-1])
                self.pulledoptIndSpin.setValue(self.hz_function[0][-1])
                self.hz0optIndSpin.setValue(self.hz_function[1][0])
                self.setTransLengthSpin.setValue((self.hz_function[0][-1] + self.hz_function[1][0] - self.hz_function[1][-1])/2)
                self.set_motors_params()
                self.timeleftLabel.setText(f"Time left: {self.core.get_time_left(self.total_to_pull, 0.0):.2f} s")
                
                self.profile_mode = "Optimized"
                self.statusBar.showMessage(f"Profile calculated successfully!")
            except Exception as e:
                print(e)
        
    def enhanced_edge_warn(self):
        if self.enhanceHZCheck.isChecked():
            text = "When this option is enabled, the puller motors stop moving during the hot zone edges. " + \
                   "In theory, it should minimize the effect of the brusher slow acceleration, which can " + \
                   "cause bumps and dips in the taper profile, increasing losses. It is also possible that " + \
                   "this effect is what enables us to see the single mode signature in the spectrogram.\n" + \
                   "Keep in mind that the total pulling time will be much greater due to the pauses."
            QMessageBox.information(self, "Enhanced edge information", text, QMessageBox.StandardButton.Ok)
            
    def set_adaptive_vel(self):
        self.set_motors_params()
        self.timeleftLabel.setText(f"Time left: {self.core.get_time_left(self.total_to_pull, 0.0):.2f} s")
    
    # Menu functions
    def action_save_data(self):
        file = QFileDialog.getSaveFileName(self, "Save data prefix", self.last_dir)
        file_prefix = file[0]
        
        self.last_dir = os.path.dirname(os.path.realpath(file_prefix)).replace("\\", "/")
        
        self.save_data(file_prefix)
        
    def clear_autosaved_data(self):
        text1 = "This will delete ALL data that was automatically saved whenever a " + \
                "pulling process finished.\n" + \
                "This means ALL data, even from many years ago, or generated by other people.\n" + \
                "Manually saved files will not be touched, if they were saved in " + \
                "a directory outside this program working dir.\n\n" + \
                "Are you ABSOLUTELY sure about this?"
        reply = QMessageBox.critical(self, "Attention!", text1, QMessageBox.StandardButton.Yes, 
                                     QMessageBox.StandardButton.No)
        if reply == QMessageBox.StandardButton.Yes:
            text1 = "Are you REALLY, REALLY, SUPER ABSOLUTELY MEGA SURE about this?\n" + \
                    "There is no going back! No recycle bin! Everything will be gone for good!"
            reply = QMessageBox.critical(self, "Attention!", text1, QMessageBox.StandardButton.Yes, 
                                     QMessageBox.StandardButton.No)
            if reply == QMessageBox.StandardButton.Yes:
                data_folder = f"{rootpath}/data_bckp"
                if os.path.exists(data_folder):
                    shutil.rmtree(data_folder)
                    QMessageBox.information(self, "Bye bye data", "Done, all data deleted.")
                else:
                    QMessageBox.information(self, "No data", "No previous data found, so nothing was deleted.")
        
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

        
    