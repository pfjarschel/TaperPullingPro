# Import PyQt6 stuff
from PyQt6 import uic
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *

# Import other stuff
import os
from matplotlib import pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar


# Load UI files
thispath = os.path.dirname(os.path.realpath(__file__))
respath = f"{thispath}/../resources"
rootpath = f"{thispath}/.."
currpath = os.getcwd()
os.chdir(rootpath)
FormUI, WindowUI = uic.loadUiType(f"{respath}/mainwindow.ui")
# os.chdir(currpath)


class MainWindow(FormUI, WindowUI):
    busy = False
    wait_switch = False
    main_to = 1  # ms
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.setupUi(self)
        self.show()
        self.initialize()
        self.configUi()
        self.enableControls()
        
    def configUi(self):
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
        
        
        # Settings
        if not os.path.isfile("PyTaper_factory_settings.json"):
            self.saveSettings("PyTaper_factory_settings.json")
            
        if not os.path.isfile("PyTaper_default_settings.json"):
            self.saveSettings("PyTaper_default_settings.json")
        else:
            self.loadDefaultSettings()
            
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
    
    def initialize(self):
        pass
    
    def saveSettings(self, file):
        pass
    
    def loadSettings(self):
        pass
    
    def loadDefaultSettings(self):
        pass
    
    def enableControls(self):
        widgets = self.findChildren(QWidget)
        for i in range(len(widgets)):
            widgets[i].setEnabled(True)
        self.stopBut.setEnabled(False)
        
    def disableControls(self):
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
        
    def resetPullStats(self):
        pass

    def mainLoop(self):
        pass
    
    def initLoop(self):
        pass
    
    def flameholdLoop(self):
        pass
    
    def pullLoop(self):
        pass
    
    def go2zeroLoop(self):
        pass
    
    def recalc_from_diam(self):
        pass
    
    def recalc_from_pulled(self):
        pass