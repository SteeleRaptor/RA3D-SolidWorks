from tkinter import *
import tkinter.ttk as ttk
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
        # (TEMP) Set window as topmost
        self.attributes('-topmost', True)
        self.updateDelay = 500 # Delay between update function calls in milliseconds
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
        self.root.terminalPrint("GUI created")
        self.serialController.refreshCOMPorts()
        # Set up a call to the update function after updateDelay milliseconds
        self.after(self.updateDelay, self.update)

    def createWidgets(self):
        # ==========| Serial Frame |==========
        self.serialFrame = Frame(self.root, bg="#FF0000", highlightthickness=2, highlightbackground="#000000")
        self.serialFrame.grid(row=0, column=0, padx=5, pady=5, sticky=W+N+E+S)
        # Create label for Serial Frame
        self.serialLabel = Label(self.serialFrame, text="Serial Frame")
        self.serialLabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        # Refresh button
        self.refreshCOMButton = Button(self.serialFrame, text="⟳", command=self.serialController.refreshCOMPorts, width=2)
        self.refreshCOMButton.grid(row=1, column=0, padx=5, pady=5)
        # Create dropdown list of all serial COM ports
        self.portList = [] # Start with blank list
        self.portSelection = StringVar(value="Select Port") # Create a StringVar to hold the current dropdown selection
        self.portSelection.trace("w", self.portSelectionChanged) # Connect portSelection changing to a function call to detect when the selected option changes
        self.portDropdown = ttk.Combobox(self.serialFrame, width=10, textvariable=self.portSelection, state="readonly") # Create the dropdown
        self.portDropdown["values"] = self.portList
        self.portDropdown.grid(row=1, column=1, padx=5, pady=5)
        
        # Create button for connecting to port
        self.connectButton = Button(self.serialFrame, text="Connect", command=self.serialController.serialConnect, width=10, state="disabled")
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
        self.calibrateButton = Button(self.controlFrame, text="Calibrate", command=self.armController.startArmCalibration, width=10)
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
        self.sendCommandButton = Button(self.moveFrame, text="Send Command", command=self.armController.prepMLCommand)
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

        self.textScroll = Scrollbar(self.printingFrame, orient="vertical")
        self.textBox = Text(self.printingFrame,
                            wrap=NONE,
                            width=50,
                            height=20,
                            yscrollcommand=self.textScroll.set,
                            state="disabled"
                            )
        self.textScroll.config(command=self.textBox.yview)
        self.textBox.grid(row=5, column=0, columnspan=5, padx=(5,0), pady=5)
        self.textScroll.grid(row=5, column=6, padx=(0, 5), pady=5, sticky=N+S)

        # ==========| Terminal Panel |==========
        self.termFrame = Frame(self.root, bg="#00FFFF", highlightthickness=2, highlightbackground="#000000")
        self.termFrame.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky=W+E+N+S)

        self.termVertScroll = Scrollbar(self.termFrame, orient="vertical")
        self.termHorzScroll = Scrollbar(self.termFrame, orient="horizontal")

        self.terminal = Text(self.termFrame,
                            wrap=NONE,
                            width=80,
                            height=10,
                            yscrollcommand=self.termVertScroll.set,
                            xscrollcommand=self.termHorzScroll.set,
                            state="disabled"
                            )
        
        
        self.termVertScroll.pack(side=RIGHT, fill=Y)
        self.termVertScroll.config(command=self.terminal.yview)
        
        self.termHorzScroll.pack(side=BOTTOM, fill=X)
        self.termHorzScroll.config(command=self.terminal.xview)
        #self.terminal.grid(row=1, column=0, columnspan=5, padx=5, pady=5)
        self.terminal.pack(fill=BOTH)

    def update(self):
        #print("\nUpdate function called:")

        # ==========| SerialController |==========

        # ===========| ArmController |============
        # If arm calibration is in progress, call the calibration update function
        if self.armController.calibrationInProgress:
            self.armController.calibrateArmUpdate()
        if self.armController.awaitingMoveResponse:
            self.armController.moveUpdate()

        # ==========| PrintController |==========


        # Set up another call to the update function after updateDelay milliseconds
        self.after(self.updateDelay, self.update)

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

    # Used to print to the in window terminal
    def terminalPrint(self, message):
        self.terminal.config(state="normal") # Need to enable to modify
        self.terminal.insert(END, f"{datetime.now().now()}| {message}\n") # Print the message with the current time fixated at the front
        self.terminal.config(state="disabled") # Disable again to avoid user changes
        self.terminal.see("end") # Forces terminal to autoscroll with new text. Probably want to make this a toggle option

if __name__ == "__main__":
    app = TkWindow()
    app.mainloop()