from tkinter import *
import tkinter.ttk as ttk
import serial
import serial.tools.list_ports
from datetime import datetime

from SerialController import SerialController
from ArmController import ArmController
from PrintController import PrintController

class TkWindow(Tk):
    def __init__(self):
        Tk.__init__(self)
        self.root = self
        # Set the window title
        self.title("RA3D Control Software")
        self.attributes('-topmost', True)
        # Set the window dimensions and position on screen
        w = 675 # Window width
        h = 725 # Window height
        ws = self.winfo_screenwidth() # Get screen width
        hs = self.winfo_screenheight() # Get screen height
        x = int((ws/2) - (w/2)) # Calculate x position for window to be in the center of the screen
        y = int((hs/2) - (h/2)) # Calculate y position for window to be in the center of the screen
        self.geometry(f"{w}x{h}+{x}+{y}") # Set the width, height, x, and y values
        
        # Instantiate objects for the various controller classes
        self.serialController = SerialController(self.root)
        self.armController = ArmController(self.root, self.serialController)
        self.printController = PrintController(self.root)

        # Create and draw widgets onto the window
        self.createWidgets()
        self.refreshCOMPorts()

    def createWidgets(self):
        # ==========| Serial Frame |==========
        self.serialFrame = Frame(self.root, bg="#FF0000", highlightthickness=2, highlightbackground="#000000")
        self.serialFrame.grid(row=0, column=0, padx=5, pady=5, sticky=W+N+E+S)
        # Create label for Serial Frame
        self.serialLabel = Label(self.serialFrame, text="Serial Frame")
        self.serialLabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        # Refresh button
        self.refreshCOMButton = Button(self.serialFrame, text="⟳", command=self.refreshCOMPorts, width=2)
        self.refreshCOMButton.grid(row=1, column=0, padx=5, pady=5)
        # Create dropdown list of all serial COM ports
        self.portList = [] # Start with blank list
        self.portSelection = StringVar(value="Select Port") # Create a StringVar to hold the current dropdown selection
        self.portSelection.trace("w", self.portSelectionChanged) # Connect portSelection changing to a function call to detect when the selected option changes
        self.portDropdown = ttk.Combobox(self.serialFrame, width=10, textvariable=self.portSelection, state="readonly") # Create the dropdown
        self.portDropdown["values"] = self.portList
        self.portDropdown.grid(row=1, column=1, padx=5, pady=5)
        
        # Create button for connecting to port
        self.connectButton = Button(self.serialFrame, text="Connect", command=self.serialConnect, width=10, state="disabled")
        self.connectButton.grid(row=1, column=2, padx=5, pady=5)
        # Status label
        self.portStatusLabel = Label(self.serialFrame, text="Status: Disconnected")
        self.portStatusLabel.grid(row=2, column=0, columnspan=3, padx=5, pady=5, sticky=W)


        # ==========| Control Frame |==========
        self.controlFrame = Frame(self.root, bg="#00FF00", highlightthickness=2, highlightbackground="#000000")
        self.controlFrame.grid(row=1, column=0, padx=5, pady=5, sticky=W+N+E+S)
        # Create label fro Control Frame
        self.controlLabel = Label(self.controlFrame, text="Control Frame")
        self.controlLabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        # Create calibration button
        self.calibrateButton = Button(self.controlFrame, text="Calibrate", command=self.armController.calibrateArm, width=10)
        self.calibrateButton.grid(row=1, column=0, padx=5, pady=5, sticky=W)
        self.extraCalibrateButton = Button(self.controlFrame, text="Extra Calibrates", command=self.extraCalibrate, width=11)
        self.extraCalibrateButton.grid(row=1, column=1, padx=5, pady=5, sticky=W)

        # Offsets
        self.calOffsetFrame = Frame(self.controlFrame)
        self.calOffsetFrame.grid(row=2, column=0, columnspan=3, padx=5, pady=5, sticky=W+E)
        self.offsetLabel = Label(self.calOffsetFrame, text="Joint Offsets:")
        self.offsetLabel.grid(row=0, column=0, columnspan=5, padx=5, pady=5, sticky=W)
        # Make the widgets
        self.J1OffsetLabel = Label(self.calOffsetFrame,text="J1:")
        self.J1OffsetEntry = Entry(self.calOffsetFrame, width=4)
        self.J2OffsetLabel = Label(self.calOffsetFrame,text="J2:")
        self.J2OffsetEntry = Entry(self.calOffsetFrame, width=4)
        self.J3OffsetLabel = Label(self.calOffsetFrame,text="J3:")
        self.J3OffsetEntry = Entry(self.calOffsetFrame, width=4)
        self.J4OffsetLabel = Label(self.calOffsetFrame,text="J4:")
        self.J4OffsetEntry = Entry(self.calOffsetFrame, width=4)
        self.J5OffsetLabel = Label(self.calOffsetFrame,text="J5:")
        self.J5OffsetEntry = Entry(self.calOffsetFrame, width=4)
        self.J6OffsetLabel = Label(self.calOffsetFrame,text="J6:")
        self.J6OffsetEntry = Entry(self.calOffsetFrame, width=4)
        # Grid the widgets
        self.J1OffsetLabel.grid(row=1, column=0, padx=5, pady=5)
        self.J1OffsetEntry.grid(row=1, column=1, padx=5, pady=5)
        self.J2OffsetLabel.grid(row=1, column=2, padx=5, pady=5)
        self.J2OffsetEntry.grid(row=1, column=3, padx=5, pady=5)
        self.J3OffsetLabel.grid(row=1, column=4, padx=5, pady=5)
        self.J3OffsetEntry.grid(row=1, column=5, padx=5, pady=5)
        self.J4OffsetLabel.grid(row=2, column=0, padx=5, pady=5)
        self.J4OffsetEntry.grid(row=2, column=1, padx=5, pady=5)
        self.J5OffsetLabel.grid(row=2, column=2, padx=5, pady=5)
        self.J5OffsetEntry.grid(row=2, column=3, padx=5, pady=5)
        self.J6OffsetLabel.grid(row=2, column=4, padx=5, pady=5)
        self.J6OffsetEntry.grid(row=2, column=5, padx=5, pady=5)

        self.J1OffsetEntry.insert(0, "0")
        self.J2OffsetEntry.insert(0, "0")
        self.J3OffsetEntry.insert(0, "0")
        self.J4OffsetEntry.insert(0, "0")
        self.J5OffsetEntry.insert(0, "0")
        self.J6OffsetEntry.insert(0, "0")

        # Move Linear label
        self.moveFrame = Frame(self.controlFrame)
        self.moveFrame.grid(row=3, column=0, columnspan=3, padx=5, pady=5, sticky=W+E)
        self.moveLabel = Label(self.moveFrame, text="Move:")
        self.moveLabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        # Send command button
        self.sendCommandButton = Button(self.moveFrame, text="Send Command", command=self.sendMoveCommand)
        self.sendCommandButton.grid(row=0, column=3, columnspan=3, padx=5, pady=5)

        # Coordinate labels and text boxes
        # Create them
        self.xCoordLabel = Label(self.moveFrame, text="X:")
        self.xCoordEntry = Entry(self.moveFrame, width=4)
        self.yCoordLabel = Label(self.moveFrame, text="Y:")
        self.yCoordEntry = Entry(self.moveFrame, width=4)
        self.zCoordLabel = Label(self.moveFrame, text="Z:")
        self.zCoordEntry = Entry(self.moveFrame, width=4)
        self.RxCoordLabel = Label(self.moveFrame, text="Rx:")
        self.RxCoordEntry = Entry(self.moveFrame, width=4)
        self.RyCoordLabel = Label(self.moveFrame, text="Ry:")
        self.RyCoordEntry = Entry(self.moveFrame, width=4)
        self.RzCoordLabel = Label(self.moveFrame, text="Rz:")
        self.RzCoordEntry = Entry(self.moveFrame, width=4)
        # Display them
        self.xCoordLabel.grid(row=1, column=0, padx=5, pady=5)
        self.xCoordEntry.grid(row=1, column=1, padx=5, pady=5)
        self.yCoordLabel.grid(row=1, column=2, padx=5, pady=5)
        self.yCoordEntry.grid(row=1, column=3, padx=5, pady=5)
        self.zCoordLabel.grid(row=1, column=4, padx=5, pady=5)
        self.zCoordEntry.grid(row=1, column=5, padx=5, pady=5)
        self.RxCoordLabel.grid(row=2, column=0, padx=5, pady=5)
        self.RxCoordEntry.grid(row=2, column=1, padx=5, pady=5)
        self.RyCoordLabel.grid(row=2, column=2, padx=5, pady=5)
        self.RyCoordEntry.grid(row=2, column=3, padx=5, pady=5)
        self.RzCoordLabel.grid(row=2, column=4, padx=5, pady=5)
        self.RzCoordEntry.grid(row=2, column=5, padx=5, pady=5)

        # Current Position label
        self.currentPosFrame = Frame(self.controlFrame)
        self.currentPosFrame.grid(row=4, column=0, columnspan=3, padx=5, pady=5, sticky=W+E)
        self.currentPosLabel = Label(self.currentPosFrame, text="Current:")
        self.currentPosLabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        # Current position coordinate labels
        # Create them
        self.xCurCoordLabel = Label(self.currentPosFrame, text="X:")
        self.xCurCoord = Label(self.currentPosFrame, text="xxx")
        self.yCurCoordLabel = Label(self.currentPosFrame, text="Y:")
        self.yCurCoord = Label(self.currentPosFrame, text="xxx")
        self.zCurCoordLabel = Label(self.currentPosFrame, text="Z:")
        self.zCurCoord = Label(self.currentPosFrame, text="xxx")
        self.RxCurCoordLabel = Label(self.currentPosFrame, text="Rx:")
        self.RxCurCoord = Label(self.currentPosFrame, text="xxx")
        self.RyCurCoordLabel = Label(self.currentPosFrame, text="Ry:")
        self.RyCurCoord = Label(self.currentPosFrame, text="xxx")
        self.RzCurCoordLabel = Label(self.currentPosFrame, text="Rz:")
        self.RzCurCoord = Label(self.currentPosFrame, text="xxx")
        # Display them
        self.xCurCoordLabel.grid(row=1, column=0, padx=5, pady=5)
        self.xCurCoord.grid(row=1, column=1, padx=5, pady=5)
        self.yCurCoordLabel.grid(row=1, column=2, padx=5, pady=5)
        self.yCurCoord.grid(row=1, column=3, padx=5, pady=5)
        self.zCurCoordLabel.grid(row=1, column=4, padx=5, pady=5)
        self.zCurCoord.grid(row=1, column=5, padx=5, pady=5)
        self.RxCurCoordLabel.grid(row=2, column=0, padx=5, pady=5)
        self.RxCurCoord.grid(row=2, column=1, padx=5, pady=5)
        self.RyCurCoordLabel.grid(row=2, column=2, padx=5, pady=5)
        self.RyCurCoord.grid(row=2, column=3, padx=5, pady=5)
        self.RzCurCoordLabel.grid(row=2, column=4, padx=5, pady=5)
        self.RzCurCoord.grid(row=2, column=5, padx=5, pady=5)


        # ==========| Printing Frame |==========
        self.printingFrame = Frame(self.root, bg="#0000FF", highlightthickness=2, highlightbackground="#000000")
        self.printingFrame.grid(row=0, rowspan=2, column=1, padx=5, pady=5, sticky=W+N+E+S)
        # Printing frame label
        self.printingLabel = Label(self.printingFrame, text="Print Panel")
        self.printingLabel.grid(row=0, column=0, padx=5, pady=5, sticky=W)
        # Select file button
        self.selectFileButton = Button(self.printingFrame, text="Select File", width=10, command=self.printController.selectFile)
        self.selectFileButton.grid(row=1, column=0, padx=5, pady=5)
        # Selected file label
        self.selectedFileLabel = Label(self.printingFrame, text="Please select file")
        self.selectedFileLabel.grid(row=1, column=1, columnspan=2, padx=5, pady=5, sticky=W)
        # Start button
        self.startPrintButton = Button(self.printingFrame, text="Start", width=10, command=self.printController.startPrint, state="disabled")
        self.startPrintButton.grid(row=2, column=0, padx=5, pady=5)
        self.stepPrintButton = Button(self.printingFrame, text="Step", width=10, command=self.printController.stepPrint, state="disabled")
        self.stepPrintButton.grid(row=2, column=1, padx=5, pady=5)
        # Pause button
        self.pausePrintButton = Button(self.printingFrame, text="Pause", width=10, command=self.printController.pausePrint, state="disabled")
        self.pausePrintButton.grid(row=2, column=2, padx=5, pady=5)
        # Cancel button
        self.cancelPrintButton = Button(self.printingFrame, text="Cancel", width=10, command=self.printController.cancelPrint, state="disabled")
        self.cancelPrintButton.grid(row=2, column=3, padx=5, pady=5)
        # Progress Label
        self.progressLabel = Label(self.printingFrame, text="Progress:")
        self.progressLabel.grid(row=3, column=0, padx=5, pady=5)
        # Progress bar
        self.progressBar = ttk.Progressbar(self.printingFrame, orient=HORIZONTAL, length=300, mode="determinate")
        self.progressBar.grid(row=4, column=0, columnspan=3, padx=5, pady=5)
        self.progressBar['value'] = 40

        self.textBox = Text(self.printingFrame,
                            wrap=NONE,
                            undo=True,
                            tabs=20,
                            width=50,
                            height=10,
                            state="disabled"
                            )
        self.textBox.grid(row=5, column=0, columnspan=5, padx=5, pady=5)

        # ==========| Terminal Panel |==========
        self.termFrame = Frame(self.root, bg="#00FFFF", highlightthickness=2, highlightbackground="#000000")
        self.termFrame.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky=W+E+N+S)
        #self.termLabel = Label(self.termFrame, text="Debug Terminal")
        #self.termLabel.grid(row=0, column=0, sticky=W)
        #self.termLabel.pack()

        self.terminal = Text(self.termFrame,
                            wrap=NONE,
                            width=80,
                            height=10,
                            state="disabled"
                            )
        
        self.termVertScroll = Scrollbar(self.termFrame)
        self.termVertScroll.pack(side=RIGHT, fill=Y)
        self.termVertScroll.config(command=self.terminal.yview)
        self.termHorzScroll = Scrollbar(self.termFrame)
        self.termHorzScroll.pack(side=BOTTOM, fill=X)
        self.termHorzScroll.config(command=self.terminal.xview)
        #self.terminal.grid(row=1, column=0, columnspan=5, padx=5, pady=5)
        self.terminal.pack(fill=BOTH)

        


        self.terminalPrint("GUI Created")

    def refreshCOMPorts(self):
        self.portList = self.getCOMPorts() # Create options list from found COM ports
        self.portSelection.set("Select Port") # Create a StringVar to hold the current dropdown selection
        self.portDropdown["values"] = self.portList
        self.portDropdown["textvariable"] = self.portSelection

    # Returns a list of all COM ports with a device connected
    def getCOMPorts(self):
        ports = list(serial.tools.list_ports.comports())
        returnList = []
        if not ports:
            self.terminalPrint("No serial ports w/ connected devices found")
            print("No serial ports w/ connected devices found")
        else:
            self.terminalPrint(f"Found {len(ports)} connected device(s):\n")
            print(f"Found {len(ports)} connected device(s):\n")
            for port in ports:
                self.terminalPrint(f"Name: {port.device}\nDescription: {port.description}\nID: {port.hwid}\n\n")
                print(f"Name: {port.device}\nDescription: {port.description}\nID: {port.hwid}\n\n")
                returnList.append(port.device)
        return returnList

    # Called whenever the selection in the port dropdown is changed
    def portSelectionChanged(self, *args):
        print("Port Selection Changed")
        # Check if the selected port is the default "Select Port" text
        if self.portSelection.get() == "Select Port":
            # If so, disable the connect button to avoid connecting to nothing
            self.connectButton.config(state="disabled")
        else:
            # If not, enable the connect button
            self.connectButton.config(state="normal")

    # Handles the "Connect/Disconnect" button being pressed to connect or disconnect the port
    def serialConnect(self):
        self.terminalPrint("serialConnect()")
        # Check if the serial controller is already connected to a board
        if self.serialController.boardConnected == False:
            # If not, connect to the port currently selected in the port dropdown
            self.terminalPrint("Attempting port connection")
            self.serialController.connectPort(self.portSelection.get())
            self.terminalPrint(f"Connection Status: {self.serialController.boardConnected}")
            print(f"Connection Status: {self.serialController.boardConnected}")
            # Perform a check to see if board was actually connected to
            if self.serialController.boardConnected:
                self.connectButton.config(text="Disconnect") # Change button text
                self.portDropdown.config(state="disabled") # Disable port dropdown
                self.refreshCOMButton.config(state="disabled") # Disable refresh button
                self.portStatusLabel.config(text="Status: Connected") # Change port status text
        else:
            # If so, disconnect from the port
            self.terminalPrint("Disconnecting from port")
            self.serialController.disconnectPort()
            print(f"Connection Status: {self.serialController.boardConnected}")
            self.terminalPrint(f"Connection Status: {self.serialController.boardConnected}")
            self.connectButton.config(text="Connect") # Change button text
            self.portDropdown.config(state="readonly") # Enable port dropdown
            self.refreshCOMButton.config(state="normal") # Enable refresh button
            self.portStatusLabel.config(text="Status: Disconnected") # Change port status text

    def sendMoveCommand(self):
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
        
    def extraCalibrate(self):
        print("extraCalibrate")
        self.terminalPrint("Opening extra calibrations window")
        # Create another window for extra calibration
        self.extraCalibrateTop = Toplevel(self.root)
        # Create buttons for J1-6 calibration
        self.calJ1Button = Button(self.extraCalibrateTop, text="Cal J1", command=lambda: self.armController.calibrateJoints(1, 0, 0, 0, 0, 0))
        self.calJ2Button = Button(self.extraCalibrateTop, text="Cal J2", command=lambda: self.armController.calibrateJoints(0, 1, 0, 0, 0, 0))
        self.calJ3Button = Button(self.extraCalibrateTop, text="Cal J3", command=lambda: self.armController.calibrateJoints(0, 0, 1, 0, 0, 0))
        self.calJ4Button = Button(self.extraCalibrateTop, text="Cal J4", command=lambda: self.armController.calibrateJoints(0, 0, 0, 1, 0, 0))
        self.calJ5Button = Button(self.extraCalibrateTop, text="Cal J5", command=lambda: self.armController.calibrateJoints(0, 0, 0, 0, 1, 0))
        self.calJ6Button = Button(self.extraCalibrateTop, text="Cal J6", command=lambda: self.armController.calibrateJoints(0, 0, 0, 0, 0, 1))

        # Place buttons on the window
        self.calJ1Button.grid(row=0, column=0, padx=5, pady=5)
        self.calJ2Button.grid(row=0, column=1, padx=5, pady=5)
        self.calJ3Button.grid(row=1, column=0, padx=5, pady=5)
        self.calJ4Button.grid(row=1, column=1, padx=5, pady=5)
        self.calJ5Button.grid(row=2, column=0, padx=5, pady=5)
        self.calJ6Button.grid(row=2, column=1, padx=5, pady=5)

    def terminalPrint(self, message):
        self.terminal.config(state="normal") # Need to enable to modify
        self.terminal.insert(END, f"{datetime.now().now()}: {message}\n") # Print the message with the current time fixated at the front
        self.terminal.config(state="disabled") # Disable again to avoid user changes

if __name__ == "__main__":
    app = TkWindow()
    app.mainloop()