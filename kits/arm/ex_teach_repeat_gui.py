'''
GUI - HEBI 6DOF ARM EASY Teach and Repeat

Works with latest Python API
'''

# Importing the needed libraries

import hebi
import numpy as np
from tkinter import * # Tkinter is the built in PYTHON Animation Program
from time import sleep
from hebi import arm as arm_api
from hebi.util import create_mobile_io


class DataStruct:
  def __init__(self):
    self.width: int
    self.height: int
    self.timerDelay: int
    self.margin: float
    self.buttonList: list[str]

def mousePressed(event, data: DataStruct):
    buttonCheck(event, data)

def keyPressed(event, data: DataStruct):
    pass


BUTTON_WIDTH = 150
BUTTON_HEIGHT = 75


#----- The Start of the Graphical Interface ------#
def init(data: DataStruct):
    data.margin = 20 # Spacing for every new waypoint
    data.timerDelay = 5
  
    # Create a list of Buttons to be drawn on the canvas
    data.buttonList = ["B1 \n Add Stop Waypoint ",
              "B2 \n Add Stop \n Toggle the gripper",
              "B3 \n Add Flow Waypoint",
              "B5 \n Toggle \n Training/Playback",
              "B6 \n Clear Waypoints",
              "B8 \n Quit"]
    data.buttonColors =["yellow"] * len(data.buttonList)

    data.buttonPress = np.zeros((len(data.buttonList), 1))
  
    # Sizing values for the buttons
    data.buttonwidth = 150
    data.buttonheight = 75

    data.buttonPos = createButtonPos(data)

    # Initializing list to keep track of waypoints
    data.run_mode = "training"
    data.goal = np.empty((0,3), dtype=np.float64)
    data.time = np.array([])
    data.gripper = np.array([])


def run(width, height):
    def redrawAllWrapper(canvas: Canvas, data: DataStruct):
        canvas.delete(ALL)
        redrawAll(canvas, data)
        canvas.update()

    def mousePressedWrapper(event, canvas: Canvas, data: DataStruct):
        mousePressed(event, data)
        redrawAllWrapper(canvas, data)

    def keyPressedWrapper(event, canvas: Canvas, data: DataStruct):
        keyPressed(event, data)
        redrawAllWrapper(canvas, data)

    def timerFiredWrapper(canvas: Canvas, data: DataStruct):
        abort_flag = timerFired(data)
        if abort_flag == True:
            print("Timer Fire Wrapper Abort")
        redrawAllWrapper(canvas, data)
        # pause, then call timerFired again
        canvas.after(data.timerDelay, timerFiredWrapper, canvas, data)
        # Set up data and call init
        return abort_flag

    data = DataStruct()
    data.width = width
    data.height = height
    data.timerDelay = 5 # milliseconds
    init(data)
    # create the root and the canvas
    root = Tk()
    canvas = Canvas(root, width=data.width, height=data.height)
    canvas.pack() # Drawing the canvas

    root.bind("<Button-1>", lambda event: mousePressedWrapper(event, canvas, data))
    root.bind("<Key>", lambda event: keyPressedWrapper(event, canvas, data))
    abort_flag = timerFiredWrapper(canvas, data)
    if abort_flag == True:
        root.destroy()
    # and launch the app
    # Adjust size
    root.geometry("800x600")
     
    # set minimum window size value
    root.minsize(data.width, data.height)
     
    # set maximum window size value
    root.maxsize(data.width, data.height)
    root.mainloop()  # blocks until window is closed


if __name__ == "__main__":
    lookup = hebi.Lookup()
    # Give the Lookup process 2 seconds to discover modules
    sleep(2)
    
    # Print all the modules on the network
    print('Modules found on network:')
    for entry in lookup.entrylist:
      print(f'{entry.family} | {entry.name}')
    
    # Arm Setup
    arm_family   = "Rosie" # Change this if your modules are on a different family name
    module_names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']
    hrdf_file    = "hrdf/A-2085-06G.hrdf"
    gains_file   = "gains/A-2085-06.xml" # make sure this file in the same folder as this program
    
    arm = arm_api.create([arm_family],
                         names=module_names,
                         hrdf_file=hrdf_file)
    arm.load_gains(gains_file)
    
    
    # Add the gripper 
    gripper_family = arm_family
    gripper_name   = 'gripperSpool'
    
    gripper_group = lookup.get_group_from_names([gripper_family], [gripper_name])
    while gripper_group is None:
        print('Looking for gripper...')
        sleep(1)
        gripper_group = lookup.get_group_from_names([gripper_family], [gripper_name])
    
    gripper = arm_api.Gripper(gripper_group, -5, 1)
    gripper.load_gains("gains/gripper_spool_gains.xml")
    arm.set_end_effector(gripper)
    
    
    # Create mobileIO object
    """
    phone_name = "mobileIO"
    print('Waiting for Mobile IO device to come online...')
    m = create_mobile_io(lookup, arm_family, phone_name)
    if m is None:
      raise RuntimeError("Could not find Mobile IO device")
    m.set_led_color("blue") # as we start in grav comp
    m.clear_text() # Clear any garbage on screen
    m.update()
    """
    
    # Demo Variables
    abort_flag = False
    pending_goal = False
    run_mode = "training"
    goal = arm_api.Goal(arm.size)
    
    # Printing Instructions
    instructions = """B1 - Add waypoint (stop)
    B2 - Add waypoint (stop) and toggle the gripper
    B3 - Add waypoint (flow)
    B5 - Toggle training/playback
    B6 - Clear waypoints
    A3 - Up/down for longer/shorter time to waypoint
    B8 - Quit
    """
    print(instructions)
    
    #######################
    ## Main Control Loop ##
    #######################
    
    # Set Resolution of the canvas
    resx = 800
    resy = 600

    def createButtonPos(data):
        # Creating the list of rectangle coords
        buttonRows = len(data.buttonList)
        pointsDefRect = 4

        buttonPos = np.zeros((buttonRows, pointsDefRect))
        # Displaying all the buttons on the screen
        for row in range(buttonRows):
            (x1,y1) = (3*data.width/4, (data.margin) + ((data.margin + BUTTON_HEIGHT) * row))
            (x2,y2) = (3*data.width/4 + BUTTON_WIDTH, (data.margin + BUTTON_HEIGHT) + (data.margin + BUTTON_HEIGHT)*row)
            buttonPos[row] = [x1,y1,x2,y2]

        return buttonPos
    
    # Checks Mouse Pointer on the Screen
    def buttonCheck(event,data):
        mx = event.x
        my = event.y
        for row, pos in enumerate(data.buttonPos):
            (x1,y1,x2,y2) = pos 
            if (x1 < mx < x2) and (y1 < my < y2):
                data.buttonPress[row] = 1
      
    
    def timerFired(data):
        #updatewaypoints(data)
        arm.update()
    
        # Set all the button colors to yellow
        data.buttonColors =["yellow"]*len(data.buttonList)
    
        # Shutting Down the Arm
        if data.buttonPress[5] == 1: # "ToOn"
            print("Shutting Down the Program")
            data.buttonColors[5] = "blue"
            abort_flag = True
            quit()
    
        # Keep Button 5 as blue when the code is in playback mode
        if data.run_mode == "playback":
            data.buttonColors[3] = "blue"
    
        if data.run_mode == "training":
            # B1 add waypoint (stop)
            if data.buttonPress[0] == 1: # "ToOn"
                print("Stop waypoint added")
                goal.add_waypoint(t=3.0, position=arm.last_feedback.position, aux=gripper.state, velocity=[0]*arm.size)
                data.time = np.append(data.time, 3.0)
                data.gripper = np.append(data.gripper, 0)
                data.goal=np.append(data.goal, [arm.FK(arm.last_feedback.position)], axis = 0)
                data.buttonColors[0] = "blue"
    
            # B2 add waypoint (stop) and toggle the gripper
            if data.buttonPress[1] == 1: # "ToOn"
                # Add 2 waypoints to allow the gripper to open or close
                print("Stop waypoint added and gripper toggled")
                position = arm.last_feedback.position
                goal.add_waypoint(t= 3.0, position=position, aux=gripper.state, velocity=[0]*arm.size)
                gripper.toggle()
                goal.add_waypoint(t=2.0, position=position, aux=gripper.state, velocity=[0]*arm.size)
    
                data.time = np.append(data.time, 5.0)
                data.gripper = np.append(data.gripper, 1)
                data.goal=np.append(data.goal, [arm.FK(arm.last_feedback.position)], axis = 0)
                data.buttonColors[1] = "blue"
    
            # B3 add waypoint (flow)
            if data.buttonPress[2] == 1: # "ToOn"
                print("Flow waypoint added")
                goal.add_waypoint(t= 3.0, position=arm.last_feedback.position, aux=gripper.state)
                data.time = np.append(data.time, 3.0)
                data.gripper = np.append(data.gripper, 0)
                data.goal=np.append(data.goal, [arm.FK(arm.last_feedback.position)], axis = 0)
                data.buttonColors[2] = "blue"
    
            # B5 toggle training/playback
            if data.buttonPress[3] == 1: # "ToOn"
                # Check for more than 2 waypoints
                if goal.waypoint_count > 1:
                    print("Transitioning to playback mode")
                    data.run_mode = "playback"
                    arm.set_goal(goal)
                else:
                    print("At least two waypoints are needed")
                data.buttonColors[3] = "blue"
    
            # B6 clear waypoints
            if data.buttonPress[4] == 1: # "ToOn"
                print("Waypoints cleared")
                goal.clear()
                data.goal = np.zeros((1,3), dtype=np.float64)
                data.time = np.array([])
                data.gripper = np.array([])
                data.buttonColors[4] = "blue"
    
        elif data.run_mode == "playback":    
            # B5 toggle training/playback
            if data.buttonPress[3] == 1: # "ToOn"
                print("Transitioning to training mode")
                data.run_mode = "training"
                arm.cancel_goal()
    
            # replay through the path again once the goal has been reached
            if arm.at_goal:
                arm.set_goal(goal)
        data.buttonPress = np.zeros((data.buttonRows, 1)) #Reset the buttonClick
        arm.send()
    
    def drawText(canvas: Canvas, data: DataStruct):
        # Draws First Row Text
        canvas.create_text(5, data.margin, text="Waypoints",anchor= SW)
        canvas.create_text(150, data.margin, text="Joints",anchor= SW)
        canvas.create_text(300, data.margin, text="Time",anchor= SW)
        canvas.create_text(400, data.margin, text="Gripper",anchor= SW)
    
        # Displaying all the buttons on the screen
        for row, pos in enumerate(data.buttonPos):
            (x1,y1,x2,y2) = pos 
            canvas.create_rectangle(x1,y1,x2,y2, fill= data.buttonColors[row])
            canvas.create_text(x1+(x2 - x1)/2, y1+(y2 - y1)/2, text= data.buttonList[row], justify = CENTER)

        # Adding Waypoints information
        # Draw added goal positions
        numWayPoints = len(data.time)
        for row in range(numWayPoints):
            canvas.create_text(5, data.margin * (row+2), text="%d" % int(row + 1),anchor= SW)
            goalList = data.goal[row]
            goalList = np.around(goalList, 2)
            goalStr = str(goalList)
            canvas.create_text(150, data.margin * (row+2), text= goalStr,anchor= SW)
            canvas.create_text(300, data.margin * (row+2), text="%0.1f" % float(data.time[row]),anchor= SW)
            canvas.create_text(400, data.margin * (row+2), text="%d" % data.gripper[row] ,anchor= SW)
    
    def redrawAll(canvas, data):
        drawText(canvas, data)
    
    run(resx, resy)
    