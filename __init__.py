import main

# TODO: Not sure if these are still relevant
__all__ = ["TaperShape", "TaperPullingUI", "TaperPullingData", "TaperPullingDAQ", "TaperPullingMotors"]

def run():
    app, window = main.create_ui()
    # mainwindow.show()  # TODO: Really needed?
    
    # Run application and return status when finished.
    return app.exec()