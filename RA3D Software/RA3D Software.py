from tkinter import *
import tkinter.ttk as ttk
import serial
import serial.tools.list_ports

from SerialController import SerialController
from ArmController import ArmController

class TkWindow(Tk):
    def __init__(self):
        Tk.__init__(self)
        self.root = self
        # Set the window title
        self.title("RA3D Control Software")
        # Set the window dimensions and position on screen
        w = 700 # Window width
        h = 510 # Window height
        ws = self.winfo_screenwidth() # Get screen width
        hs = self.winfo_screenheight() # Get screen height
        x = int((ws/2) - (w/2)) # Calculate x position for window to be in the center of the screen
        y = int((hs/2) - (h/2)) # Calculate y position for window to be in the center of the screen
        self.geometry(f"{w}x{h}+{x}+{y}") # Set the width, height, x, and y values

        # Instantiate objects for the various controller classes
        self.serialController = SerialController(self.root)
        self.armController = ArmController(self.root, self.serialController)

        # Create and draw widgets onto the window
        self.createWidgets()

    def createWidgets(self):
        # Serial Port Connection/Disconnection Frame
        self.serialFrame = Frame(self.root, bg="#FF0000", highlightthickness=2, highlightbackground="#000000")
        self.serialFrame.grid(row=1, column=1, padx=5, pady=5, sticky=W+N+E+S)
        # Create label for Serial Frame
        self.serialLabel = Label(self.serialFrame, text="Serial Frame")
        self.serialLabel.grid(row=1, column=1, padx=5, pady=5)
        # Create dropdown list of all serial COM ports
        self.portList = ["Select Port"] # Default value of "Select Port"
        self.portList = self.portList + self.getCOMPorts() # Concatenate all found COM ports to portList
        self.portSelection = StringVar(value=self.portList[0]) # Create a StringVar to hold the current dropdown selection
        self.portSelection.trace("w", self.portSelectionChanged)
        self.portDropdown = OptionMenu(self.serialFrame, self.portSelection, *self.portList) # Create the dropdown
        self.portDropdown.config(width=10) # Set the dropdown width
        self.portDropdown.grid(row=2, column=1, padx=5, pady=5)
        # Create button for connecting to port
        self.connectButton = Button(self.serialFrame, text="Connect", command=self.serialConnect, width=10, state="disabled")
        self.connectButton.grid(row=2, column=2, padx=5, pady=5)
        # Status label
        self.portStatusLabel = Label(self.serialFrame, text="Status: Disconnected")
        self.portStatusLabel.grid(row=3, column=1, columnspan=2, padx=5, pady=5, sticky=W)

        # Control Frame
        self.controlFrame = Frame(self.root, bg="#00FF00", highlightthickness=2, highlightbackground="#000000")
        self.controlFrame.grid(row=2, column=1, padx=5, pady=5, sticky=W+N+E+S)
        # Create label fro Control Frame
        self.controlLabel = Label(self.controlFrame, text="Control Frame")
        self.controlLabel.grid(row=1, column=1, columnspan=3, padx=5, pady=5)
        # Create calibration button
        self.calibrateButton = Button(self.controlFrame, text="Calibrate", command=self.armController.calibrateArm, width=10)
        self.calibrateButton.grid(row=2, column=1, columnspan=3, padx=5, pady=5)
        # Move Linear label
        self.moveLabel = Label(self.controlFrame, text="Move:")
        self.moveLabel.grid(row=3, column=1, columnspan=2, padx=5, pady=5)
        # Send command button
        self.sendCommandButton = Button(self.controlFrame, text="Send Command", command=self.sendMoveCommand)
        self.sendCommandButton.grid(row=3, column=3, columnspan=3, padx=5, pady=5)

        # Coordinate labels and text boxes
        # Create them
        self.xCoordLabel = Label(self.controlFrame, text="X:")
        self.xCoordEntry = Entry(self.controlFrame, width=4)
        self.yCoordLabel = Label(self.controlFrame, text="Y:")
        self.yCoordEntry = Entry(self.controlFrame, width=4)
        self.zCoordLabel = Label(self.controlFrame, text="Z:")
        self.zCoordEntry = Entry(self.controlFrame, width=4)
        self.RxCoordLabel = Label(self.controlFrame, text="Rx:")
        self.RxCoordEntry = Entry(self.controlFrame, width=4)
        self.RyCoordLabel = Label(self.controlFrame, text="Ry:")
        self.RyCoordEntry = Entry(self.controlFrame, width=4)
        self.RzCoordLabel = Label(self.controlFrame, text="Rz:")
        self.RzCoordEntry = Entry(self.controlFrame, width=4)
        # Display them
        self.xCoordLabel.grid(row=4, column=1, padx=5, pady=5)
        self.xCoordEntry.grid(row=4, column=2, padx=5, pady=5)
        self.yCoordLabel.grid(row=4, column=3, padx=5, pady=5)
        self.yCoordEntry.grid(row=4, column=4, padx=5, pady=5)
        self.zCoordLabel.grid(row=4, column=5, padx=5, pady=5)
        self.zCoordEntry.grid(row=4, column=6, padx=5, pady=5)
        self.RxCoordLabel.grid(row=5, column=1, padx=5, pady=5)
        self.RxCoordEntry.grid(row=5, column=2, padx=5, pady=5)
        self.RyCoordLabel.grid(row=5, column=3, padx=5, pady=5)
        self.RyCoordEntry.grid(row=5, column=4, padx=5, pady=5)
        self.RzCoordLabel.grid(row=5, column=5, padx=5, pady=5)
        self.RzCoordEntry.grid(row=5, column=6, padx=5, pady=5)

        # Current Position label
        self.currentPosLabel = Label(self.controlFrame, text="Current:")
        self.currentPosLabel.grid(row=6, column=1, columnspan=2, padx=5, pady=5)
        # Current position coordinate labels
        # Create them
        self.xCurCoordLabel = Label(self.controlFrame, text="X:")
        self.xCurCoord = Label(self.controlFrame, text="xxx")
        self.yCurCoordLabel = Label(self.controlFrame, text="Y:")
        self.yCurCoord = Label(self.controlFrame, text="xxx")
        self.zCurCoordLabel = Label(self.controlFrame, text="Z:")
        self.zCurCoord = Label(self.controlFrame, text="xxx")
        self.RxCurCoordLabel = Label(self.controlFrame, text="Rx:")
        self.RxCurCoord = Label(self.controlFrame, text="xxx")
        self.RyCurCoordLabel = Label(self.controlFrame, text="Ry:")
        self.RyCurCoord = Label(self.controlFrame, text="xxx")
        self.RzCurCoordLabel = Label(self.controlFrame, text="Rz:")
        self.RzCurCoord = Label(self.controlFrame, text="xxx")
        # Display them
        self.xCurCoordLabel.grid(row=7, column=1, padx=5, pady=5)
        self.xCurCoord.grid(row=7, column=2, padx=5, pady=5)
        self.yCurCoordLabel.grid(row=7, column=3, padx=5, pady=5)
        self.yCurCoord.grid(row=7, column=4, padx=5, pady=5)
        self.zCurCoordLabel.grid(row=7, column=5, padx=5, pady=5)
        self.zCurCoord.grid(row=7, column=6, padx=5, pady=5)
        self.RxCurCoordLabel.grid(row=8, column=1, padx=5, pady=5)
        self.RxCurCoord.grid(row=8, column=2, padx=5, pady=5)
        self.RyCurCoordLabel.grid(row=8, column=3, padx=5, pady=5)
        self.RyCurCoord.grid(row=8, column=4, padx=5, pady=5)
        self.RzCurCoordLabel.grid(row=8, column=5, padx=5, pady=5)
        self.RzCurCoord.grid(row=8, column=6, padx=5, pady=5)

        # Printing Frame
        self.printingFrame = Frame(self.root, bg="#0000FF", highlightthickness=2, highlightbackground="#000000")
        self.printingFrame.grid(row=1, rowspan=2, column=2, padx=5, pady=5, sticky=W+N+E+S)
        # Printing frame label
        self.printingLabel = Label(self.printingFrame, text="Printing")
        self.printingLabel.grid(row=1, column=1, padx=5, pady=5)
        # Select file button
        self.selectFileButton = Button(self.printingFrame, text="Select File", width=10, command=self.selectFile)
        self.selectFileButton.grid(row=2, column=1, padx=5, pady=5)
        # Selected file label
        self.selectedFileLabel = Label(self.printingFrame, text="Please select file")
        self.selectedFileLabel.grid(row=2, column=2, columnspan=2, padx=5, pady=5, sticky=W)
        # Start button
        self.startPrintButton = Button(self.printingFrame, text="Start", width=10, command=self.startPrint)
        self.startPrintButton.grid(row=3, column=1, padx=5, pady=5)
        # Pause button
        self.pausePrintButton = Button(self.printingFrame, text="Pause", width=10, command=self.pausePrint)
        self.pausePrintButton.grid(row=3, column=2, padx=5, pady=5)
        # Cancel button
        self.cancelPrintButton = Button(self.printingFrame, text="Cancel", width=10, command=self.cancelPrint)
        self.cancelPrintButton.grid(row=3, column=3, padx=5, pady=5)
        # Progress Label
        self.progressLabel = Label(self.printingFrame, text="Progress:")
        self.progressLabel.grid(row=4, column=1, padx=5, pady=5)
        # Progress bar
        self.progressBar = ttk.Progressbar(self.printingFrame, orient=HORIZONTAL, length=300, mode="determinate")
        self.progressBar.grid(row=5, column=1, columnspan=3, padx=5, pady=5)
        self.progressBar['value'] = 40


    # Returns a list of all COM ports with a device connected
    def getCOMPorts(self):
        ports = list(serial.tools.list_ports.comports())
        returnList = []
        if not ports:
            print("No serial ports w/ connected devices found")
        else:
            print(f"Found {len(ports)} connected device(s):\n")
            for port in ports:
                print(f"Name: {port.device}\nDescription: {port.description}\nID: {port.hwid}\n\n")
                returnList.append(port.device)
        return returnList

    # Called whenever the selection in the port dropdown is changed
    def portSelectionChanged(self, *args):
        print("Port Selection Changed")
        # Check if the selected port is the "Select Port" text (first option in portList)
        if self.portSelection.get() == self.portList[0]:
            # If so, disable the connect button
            self.connectButton.config(state="disabled")
        else:
            # If not, enable the connect button
            self.connectButton.config(state="normal")

    # Handles the "Connect/Disconnect" button being pressed to connect or disconnect the port
    def serialConnect(self):
        # Check if the serial controller is already connected to a board
        if self.serialController.boardConnected == False:
            # If not, connect to the port currently selected in the port dropdown
            self.serialController.connectPort(self.portSelection.get())
            print(f"Connection Status: {self.serialController.boardConnected}")
            # Perform a check to see if board was actually connected to
            if self.serialController.boardConnected:
                self.connectButton.config(text="Disconnect") # Change button text
                self.portDropdown.config(state="disabled") # Disable port dropdown
                self.portStatusLabel.config(text="Status: Connected") # Change port status text
        else:
            # If so, disconnect from the port
            self.serialController.disconnectPort()
            print(f"Connection Status: {self.serialController.boardConnected}")
            self.connectButton.config(text="Connect") # Change button text
            self.portDropdown.config(state="normal") # Enable port dropdown
            self.portStatusLabel.config(text="Status: Disconnected") # Change port status text

    def sendMoveCommand(self):
        # First check if we are connected to device and if calibration has occurred
        if not (self.serialController.boardConnected and self.armController.armCalibrated):
            print("Cannot execute move as either board isn't connected or arm hasn't been calibrated")
            return # Exit function
        # Read the values from each entry box
        x = self.xCoordEntry.get()
        y = self.yCoordEntry.get()
        z = self.zCoordEntry.get()
        Rx = self.RxCoordEntry.get()
        Ry = self.RyCoordEntry.get()
        Rz = self.RzCoordEntry.get()
        # Check if any values are blank
        allValuesNumeric = True
        if not x.isnumeric():
            print("X is not a number")
            allValuesNumeric = False
        if not y.isnumeric():
            print("Y is not a number")
            allValuesNumeric = False
        if not z.isnumeric():
            print("Z is not a number")
            allValuesNumeric = False
        if not Rx.isnumeric():
            print("Rx is not a number")
            allValuesNumeric = False
        if not Ry.isnumeric():
            print("Ry is not a number")
            allValuesNumeric = False
        if not Rz.isnumeric():
            print("Rz is not a number")
            allValuesNumeric = False
        
        if allValuesNumeric:
            print("All values numeric, sending ML command")
            self.armController.sendML(x, y, z, Rx, Ry, Rz)
        else:
            print("ML command not sent due to a value not being a number")
        
    def selectFile(self):
        pass

    def startPrint(self):
        pass

    def pausePrint(self):
        pass

    def cancelPrint(self):
        pass

if __name__ == "__main__":
    app = TkWindow()
    app.mainloop()