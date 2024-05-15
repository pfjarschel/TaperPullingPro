from . import TaperShape
from . import TaperPullingUI
from . import TaperPullingData
from . import TaperPullingDAQ
from . import TaperPullingMotors

import sys
from PyQt6.QtWidgets import QApplication

def create_ui() -> tuple[QApplication, TaperPullingUI.MainWindow]:
    app = QApplication(sys.argv)
    mainwindow = TaperPullingUI.MainWindow()
    
    return app, mainwindow
    
def run_ui() -> int:
    app, mainwindow = create_ui()
    # mainwindow.show()  # TODO: Really needed?
    
    # Run application and return status when finished.
    return app.exec()

__all__ = ["run_ui", "create_ui", "TaperShape", "TaperPullingUI", "TaperPullingData", "TaperPullingDAQ", "TaperPullingMotors"]

