from tkinter import *
from tkinter import filedialog
import os, re, copy, threading
from ArmController import Position, Origin, MoveCommand,MoveParameters

class PrintController:
    def __init__(self, root):
        self.root = root
        #Placeholder boundaries, should be adjusted
        self.maxBoundaryY = [-100,100]
        self.maxBoundaryX = [200,500]
        self.maxBoundaryZ = [100,600]
        self.selectedFilepath = None
        self.gcodeLines = []
        self.teensyLines = []
        self.fileOpen = False
        self.printing = False
        self.printPaused = False
        self.origin = root.armController.origin
        self.calibrationCorners = [Position(430,40,145,0,90,0,None),Position(470,40,145,0,90,0,None),Position(470,-40,145,0,90,0,None),Position(430,-40,145,0,90,0,None)]
        # Parameters for printing coordinates
        #self.xBounds = [300, 500] # X Min & X Max
        #self.yBounds = [-100, 100] # Y Min & Y Max
        #self.zBounds = [100, 300] # Z Min & Z Max
        #self.xyCenter = [(self.xBounds[0] + self.xBounds[1]) / 2, (self.yBounds[0] + self.yBounds[1]) / 2]
        # Parameters used for saving the last used coordinate information
        self.lastPos = Position(None,None,None,None,None,None,self.origin)
        self.printPos = Position(None,None,None,None,None,None,self.origin)
        self.printParemeters = MoveParameters(20,5,5,15,0,"m") #10mm/s print speed
        self.lastF = 0.0
        self.lastE = 0.0
        self.currentInstruction = 0


    def selectFile(self):
        # TODO: Needs some form of garbage collection as Python holds onto the memory allocated when opening a file
        # TODO: One possibility is utilizing "yield" command (or other methods) to only read one line at a time
        
        # This list contains valid file types
        filetypes = [
            ("GCode Files", "*.gcode"),
            ("All Files", "*.*")
        ]
        # Have user select a file
        self.selectedFilepath = filedialog.askopenfilename(filetypes=filetypes)
        # Check if user actually selected a file
        if (self.selectedFilepath == ""):
            self.root.statusPrint("No file selected")
            self.root.selectedFileLabel.config(text="No file selected")
            self.root.textBox.config(state="normal")
            self.root.textBox.delete("1.0", END) # Clear text box
            self.root.textBox.config(state="disabled")
            self.gcodeLines = [""]
            self.fileOpen = False
            # Disable the buttons
            self.root.startPrintButton.config(state="disabled")
            self.root.stepPrintButton.config(state="disabled")
            self.root.pausePrintButton.config(state="disabled")
            self.root.cancelPrintButton.config(state="disabled")
            return
        # Change selectedFileLabel to have filename
        self.root.statusPrint(f"Selected \"{os.path.basename(self.selectedFilepath)}\"")
        self.root.selectedFileLabel.config(text=os.path.basename(self.selectedFilepath))
        self.fileOpen = True
        self.currentInstruction = 0 # Reset the currentInstruction counter

        # Read all lines of the file into gcodeLines
        selectedFile = open(self.selectedFilepath, "r")
        self.gcodeLines = selectedFile.readlines()
        selectedFile.close()

        # Change the text box text to be the lines of the file
        self.root.textBox.config(state="normal") # Need to enable to modify
        self.root.textBox.delete("1.0", END) # Clear text box
        for i in range(0, len(self.gcodeLines) - 1):
            self.root.textBox.insert(END, self.gcodeLines[i])
        self.root.textBox.config(state="disabled") # Disable again to avoid user changes
        # Enable the buttons
        self.root.startPrintButton.config(state="normal")
        self.root.stepPrintButton.config(state="normal")
        self.root.pausePrintButton.config(state="normal")
        self.root.cancelPrintButton.config(state="normal")

    def startPrint(self):
        if not self.origin.checkOriginSet():
            print("Orign not set, print cancelled")
            return
        

        if self.printPaused == True and self.printing == True:
            self.printPaused = False
            return

        # When starting print, reset the "last*" parameters
        self.syncOrigin()
        print(self.origin.z, "test2")
        self.printPos.origin = self.origin
        self.lastPos = Position(self.origin.x,self.origin.y,self.origin.z,0,90,0,self.origin)
        print(self.lastPos.z)
        self.lastF = 0.0
        self.lastE = 0.0
        if not self.printPaused:
            self.currentInstruction = 0
        self.printing = True

    def stepPrint(self):
        if not self.origin.checkOriginSet():
            print("Cannot print Origin not set")
            return
        
        self.root.statusPrint("Stepping print")
        # Convert a line of gcode to teensy
        if self.currentInstruction > len(self.gcodeLines) - 1:
            self.root.statusPrint("End of program reached")
            self.printing = False
            return

        lineToConvert = self.gcodeLines[self.currentInstruction] # Pull current line
        self.currentInstruction += 1 # Increment currentInstruction
        self.root.progressBar["value"] = (self.currentInstruction / len(self.gcodeLines)) * 100 # Update progress bar to match
        self.root.terminalPrint(f"Line: {lineToConvert}")# Print the line we're converting
        point = self.gcodeToTeensy(lineToConvert) # Convert line to [X, Y, Z, Rx, Ry, Rz]
        self.root.terminalPrint(f"Point: {point}") # Print the returned point list
        if point == "": # If the point is blank, don't try to send a command
            return
        # Send the command to the arm
        # TODO Find/create a better move command, consider using moveG/drivemotorsG for gcode
        self.root.armController.sendML(X=point[0], Y=point[1], Z=point[2], Rx=point[3], Ry=point[4], Rz=point[5], MoveParameters=self.printParemeters)

    def pausePrint(self):
        self.printing = False
        self.printPaused = True
    def cancelPrint(self):
        self.currentInstruction=0
        pass
    
    # Converts a GCode instruction to the instruction to send over serial
    def gcodeToTeensy(self, lineToConvert):
        if lineToConvert[0] == ';': # Line is comment
            return "" # Don't convert
        elif lineToConvert[0:3] == "G21":
            return ""
        elif lineToConvert == "\n": # Line is newline
            return "" # Don't convert
        # Actual instructions to convert
        elif lineToConvert[0:3] == "G28": # Home the printer
            self.armController.moveSafe() #Should probably implement home seperate from safe position
            return ""
        elif lineToConvert[0:3] == "G90": # Absolute positioning
            # TODO: This needs handling or removal
            return ""
        elif lineToConvert[0:2] == "G0" or lineToConvert[0:2] == "G1": # Move (treating G0 & G1 as equal)
            xMatch = re.search(r"X(-?\d+\.?\d*)", lineToConvert)
            yMatch = re.search(r"Y(-?\d+\.?\d*)", lineToConvert)
            zMatch = re.search(r"Z(-?\d+\.?\d*)", lineToConvert)
            fMatch = re.search(r"F(-?\d+\.?\d*)", lineToConvert)
            eMatch = re.search(r"E(-?\d+\.?\d*)", lineToConvert)

            x = float(xMatch.group(1)) if xMatch else None
            y = float(yMatch.group(1)) if yMatch else None
            z = float(zMatch.group(1)) if zMatch else None
            print("z read:", z)
            f = float(fMatch.group(1)) if fMatch else None
            e = float(eMatch.group(1)) if eMatch else None
            # TODO: Temporary rotation information

            
            # If GCode instruction didn't contain a parameter, pull from last saved value
            # If instruction DID contain a parameter, offset the value to put it in the build volume
            if x == None:
                x = self.lastPos.GetRelative()[0]
            if y == None:
                y = self.lastPos.GetRelative()[1]
            if z == None:
                z = self.lastPos.GetRelative()[2]
            if f == None:
                f = self.lastF
            else:
                self.lastF = f
            if e == None:
                e = self.lastE
            else:
                self.lastE = e
            self.printPos.SetRelative(x,y,z,0,90,0)
            self.lastPos = copy.deepcopy(self.printPos)
            # TODO: Additional processing to ensure X, Y, and Z are within build volume
            # TODO: Additional processing for F to control speed or something
            # Note that F is in units per minute (per LinuxCNC specifications)
            # https://linuxcnc.org/docs/html/gcode/machining-center.html#sub:feed-rate
            # TODO: Note that it is pointless to convert to strings because to send the instruction to the arm (through current methods) we give the coordinates
            # TODO: Might make a custom datatype for storing position data that can be used
            #newLine = f"MLX{x}Y{y}Z{z}Rz{Rz}Ry{Ry}Rx{Rx}J70.00J80.00J90.00Sp{self.root.armController.speed}Ac{self.root.armController.acceleration}Dc{self.root.armController.deceleration}Rm{self.root.armController.ramp}Rnd0WFLm000000Q0\n"
            #return newLine
            return self.printPos.GetAbsolute()

    # This is the main function that will loop when printing a file
    def printLoop(self):
        if self.printing == False:
            return
        if self.currentInstruction > len(self.gcodeLines) - 1:
            self.root.statusPrint("End of program reached")
            self.currentInstruction = 0
            self.printing = False
            #Move Home when complete
            self.root.armController.moveHome()
            return

        lineToConvert = self.gcodeLines[self.currentInstruction] # Pull current line
        self.currentInstruction += 1 # Increment currentInstruction
        self.root.progressBar["value"] = (self.currentInstruction / len(self.gcodeLines)) * 100 # Update progress bar to match
        self.root.terminalPrint(f"Line: {lineToConvert}")# Print the line we're converting
        point = self.gcodeToTeensy(lineToConvert) # Convert line to [X, Y, Z, Rx, Ry, Rz]
        self.root.terminalPrint(f"Point: {point}") # Print the returned point list
        if point == "": # If the point is blank, don't try to send a command
            return
        # Send the command to the arm
        # TODO Find/create a better move command, consider using moveG/drivemotorsG for gcode

        self.root.armController.sendML(X=point[0], Y=point[1], Z=point[2], Rx=point[3], Ry=point[4], Rz=point[5], MoveParameters=self.printParemeters)
    def syncOrigin(self):
        self.origin = self.root.armController.origin
    def startPrintBedCalibration(self):
        if self.origin.originSet == False or self.origin == None:
            print("Origin not set, cannot begin leveling")
            return
        self.bedCalibration = True
        self.bedCalStep = 1
        self.syncOrigin()
        self.nextBedCalibration()

    def nextBedCalibration(self):
        if self.bedCalibration == True:
            bedCalibrationThread = threading.Thread(target=self.bedCalibrationStep)
            bedCalibrationThread.start()
    def bedCalibrationStep(self):
        currentCornerPos = self.calibrationCorners[self.bedCalStep-1]
        self.root.cornerLabel.config(text=f"Current Corner: {self.bedCalStep}")
        posStep = copy.deepcopy(currentCornerPos)
        posStep.z += 100
        point =posStep.GetAbsolute()
        self.root.armController.sendMJ(X=point[0], Y=point[1], Z=point[2], Rx=point[3], Ry=point[4], Rz=point[5],moveParameters=self.printParemeters)
        while self.root.armController.awaitingMoveResponse:
            self.root.armController.moveUpdate()
        point =posStep.GetAbsolute()
        self.root.armController.sendML(X=point[0], Y=point[1], Z=point[2], Rx=point[3], Ry=point[4], Rz=point[5],moveParameters=self.printParemeters)
        posStep.z +- 50
        while self.root.armController.awaitingMoveResponse:
            self.root.armController.moveUpdate()
        point = currentCornerPos.GetAbsolute()
        self.root.armController.sendML(X=point[0], Y=point[1], Z=point[2], Rx=point[3], Ry=point[4], Rz=point[5],moveParameters=self.printParemeters)
        while self.root.armController.awaitingMoveResponse:
            self.root.armController.moveUpdate()
        self.bedCalStep += 1
        #Move To position
        #End calibration
        if self.bedCalStep == 5: #Goes to 5 after 4 is comleted
            self.bedCalibration=False
            self.root.armController.moveHome()



