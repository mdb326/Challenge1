

from comm.Serial import SerialController, DataType_Int, DataType_Float, DataType_Boolean
from joystick.JoystickManager import JoystickManager
from gui.simpleGUI import SimpleGUI

import time

##### Insert your robot's MAC ADDRESS here ####
## (you can get it by running your arduino and looking at the serial monitor for your flying drone) ##
ROBOT_MAC = "34:85:18:91:BD:B0"#"34:85:18:AB:FE:68" # "DC:54:75:D7:B3:E8"
### Insert your SERIAL PORT here ###
## may look like "COM5" in windows or "/dev/tty.usbmodem14301" in mac  #
## look in arduino for the port that your specific transeiver is connected to  ##
## Note: make sure that your serial monitor is OFF in arduino or else you will get "access is denied" error. ##
PORT = "COM4"


# For debug purposes
PRINT_JOYSTICK = False


BaseStationAddress = "" # you do not need this, just make sure your DroneMacAddress is not your base station mac address



if __name__ == "__main__":
    # Communication
    serial = SerialController(PORT, timeout=.5)  # .5-second timeout
    serial.manage_peer("A", ROBOT_MAC)
    serial.manage_peer("G", ROBOT_MAC)
    time.sleep(.05)
    serial.send_preference(ROBOT_MAC, DataType_Boolean, "zEn", True)
    serial.send_preference(ROBOT_MAC, DataType_Boolean, "yawEn", True)

    # // PID terms
    serial.send_preference(ROBOT_MAC, DataType_Float, "kpyaw", 2) #2
    serial.send_preference(ROBOT_MAC, DataType_Float, "kdyaw", .1)#.1
    serial.send_preference(ROBOT_MAC, DataType_Float, "kiyaw", 0)

    serial.send_preference(ROBOT_MAC, DataType_Float, "kpz", 0.3)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kdz", 0.6)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kiz", 0.1)
    
    # // Range terms for the integral
    serial.send_preference(ROBOT_MAC, DataType_Float, "z_int_low", 0.05)
    serial.send_preference(ROBOT_MAC, DataType_Float, "z_int_high", 0.8)
    

    # Allows the robot to read the parameters from flash memory to be used.
    serial.send_control_params(ROBOT_MAC, (0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 1, 0))

    time.sleep(.2)

    # Joystick
    joystick = JoystickManager()
    mygui = SimpleGUI()
    
    ready = 1
    old_b = 0
    old_x = 0
    dt = .1
    height = 0
    servos = 75
    tz = 0
    sum = 0
    try:
        while True:
            # Axis input: [left_vert, left_horz, right_vert, right_horz, left_trigger, right_trigger]
            # Button inputs: [A, B, X, Y]
            axis, buttons = joystick.getJoystickInputs()
            
            if buttons[3] == 1: # y stops the program
                break

            # b button is a toggle which changes the ready state
            if buttons[1] == 1 and old_b == 0: # b pauses the control
                if ready != 0:
                    ready = 0
                else:
                    ready = 1
            old_b = buttons[1]


            if PRINT_JOYSTICK:
                print(" ".join(["{:.1f}".format(num) for num in axis]), buttons)

            #### CONTROL INPUTS to the robot here #########
            sum += buttons[2] * 0.5
            sum -= buttons[0] * 0.5
            fx = 0
            fz = sum
            tz = 0
            tx = 90
            if(axis[5] > 0): 
                tz = 20
            elif axis[4] > 0:
                tz = -20
            if axis[0] > 0.1:
                fx = 1

            print(ready, fx, fz, tx, tz)
            







            
            led = -buttons[2]
            ############# End CONTROL INPUTS ###############
            sensors = serial.getSensorData()
            # print(sensors)
            if (sensors):
                mygui.update(
                    cur_yaw=sensors[1],
                    des_yaw=tz,
                    cur_height=sensors[0],
                    des_height=height,
                    battery=sensors[2],
                    distance=sensors[3],
                    connection_status=True,
                )
            # Send through serial port
            serial.send_control_params(ROBOT_MAC, (ready, fx, fz, tx, tz, led, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(dt)
            
    except KeyboardInterrupt:
        print("Stopping!")
        # Send zero input
serial.send_control_params(ROBOT_MAC, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
