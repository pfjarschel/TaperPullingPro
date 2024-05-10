# This file can be run to open the main application, but also allows the UI to be open from another script,
# using the create_ui() function

 # Import system stuff
import sys

# Import UI
from PyQt6.QtWidgets import QApplication
from . import TaperPullingUI

def create_ui() -> tuple[QApplication, TaperPullingUI.MainWindow]: 
    app = QApplication(sys.argv)
    mainwindow = TaperPullingUI.MainWindow()
    
    # TODO: address "qt.gui.imageio: libpng warning: iCCP: known incorrect sRGB profile" warnings
    
    return app, mainwindow


# Run entire application if this file is called directly
if __name__ == '__main__':
    app, mainwindow = create_ui()
    # mainwindow.show()  # TODO: Really needed?
    
    sys.exit(app.exec())
