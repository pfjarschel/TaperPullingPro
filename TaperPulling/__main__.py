# This file enables the package to run as a stand alone program (python -m TaperPulling), opening the main application

 # Import system stuff
import sys

# Import UI
from PyQt6.QtWidgets import QApplication
from .TaperPullingUI import MainWindow


# Run entire application if this package is called directly
app = QApplication(sys.argv)
mainwindow = MainWindow()
sys.exit(app.exec())
