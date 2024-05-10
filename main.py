if __name__ == '__main__':
    # Import system stuff
    import os, sys

    from PyQt6.QtWidgets import QApplication

    # File paths
    main_path = os.path.dirname(os.path.realpath(__file__))
    thisfile = os.path.basename(__file__)
    os.chdir(main_path)
    
    import TaperPullingUI
    
    app = QApplication(sys.argv)
    mainwindow = TaperPullingUI.MainWindow()
    mainwindow.show()
    
    # TODO: address "qt.gui.imageio: libpng warning: iCCP: known incorrect sRGB profile" warnings
    
    # Run program
    sys.exit(app.exec())
