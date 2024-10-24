import TaperPulling
import sys, traceback


app, mainwindow = TaperPulling.create_ui()

def global_exception_handler(exctype, value, tb):
    motors_moving = 4*[False]
    motors_moving[0] = mainwindow.core.motors.brusher.moving
    motors_moving[1] = mainwindow.core.motors.flame_io.moving
    motors_moving[2] = mainwindow.core.motors.left_puller.moving
    motors_moving[3] = mainwindow.core.motors.right_puller.moving
    if any(motors_moving):
        print()
        print("###############################################################")
        print("#  ATTENTION! AN EXCEPTION OCURRED WHILE MOTORS WERE MOVING!  #")
        print("# THE FLAME WILL MOVE BACK AND ALL OTHER MOVEMENTS WILL STOP! #")
        print("###############################################################")
        print()
        
        mainwindow.core.motors.brusher.stop()
        mainwindow.core.motors.flame_io.go_to(0.0)
        mainwindow.core.motors.left_puller.stop()
        mainwindow.core.motors.right_puller.stop()
        mainwindow.core.stop_pulling()
        
    print(f"Exception Type: {exctype}")
    print(f"Exception Value: {value}")
    print("Traceback:")
    traceback.print_tb(tb)[5]
    
sys.excepthook = global_exception_handler

print(app.exec())