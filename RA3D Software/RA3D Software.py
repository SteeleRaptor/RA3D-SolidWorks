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
        self.updateDelay = 200 # Delay between update function calls in milliseconds
        # Set the window dimensions and position on screen
        w = 1200 # Window width
        h = 515 # Window height
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
        self.createTabs()
        self.root.terminalPrint("GUI created")
        self.serialController.refreshCOMPorts()
        # Set up a call to the update function after updateDelay milliseconds
        self.after(self.updateDelay, self.update)
        

    # Creates the interface tabs
    def createTabs(self):
        # Create a notebook to manage the tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True)
        # Create the tabs
        self.printTab = Frame(self.notebook, bg="#FF0000")
        self.armTab = Frame(self.notebook, bg="#00FF00")
        self.debugTab = Frame(self.notebook, bg="#0000FF")
        self.settingsTab = Frame(self.notebook, bg="#FFFF00")
        # Put the tabs on screen
        self.printTab.pack(fill="both", expand=True)
        self.armTab.pack(fill="both", expand=True)
        self.debugTab.pack(fill="both", expand=True)
        self.settingsTab.pack(fill="both", expand=True)
        # Add the tabs to the notebook
        self.notebook.add(self.printTab, text="Printing")
        self.notebook.add(self.armTab, text="Arm Control")
        self.notebook.add(self.debugTab, text="Debug")
        self.notebook.add(self.settingsTab, text="Settings")
        # Call the various functions for creating the widgets in each tab
        self.fillPrintTab()
        self.fillArmTab()
        self.fillDebugTab()
        self.fillSettingsTab()

        # Create a status label for immediate user response
        self.statusBarFrame = Frame(self.root, height=20, bg="#FF0DEB")
        self.statusBarFrame.pack(fill="both", expand=True, side="bottom")
        self.statusLabel = Label(self.statusBarFrame, text="Status: Example")
        self.statusLabel.grid(row=0, column=0, sticky=W+N+S)

    def fillPrintTab(self):
        # ==========| File Selection Frame |==========
        self.fileSelFrame = Frame(self.printTab, highlightthickness=2, highlightbackground="#000000", width=250, height=80)
        self.fileSelFrame.grid(row=0, column=0, padx=5, pady=5, sticky=W+E+N+S)
        self.fileSelFrame.grid_propagate(False)
        # Select file button
        self.selectFileButton = Button(self.fileSelFrame, text="Select File", width=10, command=self.printController.selectFile)
        self.selectFileButton.grid(row=0, column=0, padx=5, pady=5, sticky=W)
        # Selected file label
        self.selectedFileLabel = Label(self.fileSelFrame, text="Please select file")
        self.selectedFileLabel.grid(row=1, column=0, padx=5, pady=5, sticky=W)

        # ==========| Print Control Frame |==========
        self.printControlFrame = Frame(self.printTab, highlightthickness=2, highlightbackground="#000000", width=500, height=80)
        self.printControlFrame.grid(row=0, column=1, padx=5, pady=5, sticky=W+E+N+S)
        self.printControlFrame.grid_propagate(False)
        # Start button
        self.startPrintButton = Button(self.printControlFrame, text="Start", width=10, command=self.printController.startPrint, state="disabled")
        self.startPrintButton.grid(row=0, column=0, padx=5, pady=5, sticky=N+S)
        # Step button
        self.stepPrintButton = Button(self.printControlFrame, text="Step", width=10, command=self.printController.stepPrint, state="disabled")
        self.stepPrintButton.grid(row=0, column=1, padx=5, pady=5, sticky=N+S)
        # Pause button
        self.pausePrintButton = Button(self.printControlFrame, text="Pause", width=10, command=self.printController.pausePrint, state="disabled")
        self.pausePrintButton.grid(row=0, column=2, padx=5, pady=5, sticky=N+S)
        # Cancel button
        self.cancelPrintButton = Button(self.printControlFrame, text="Cancel", width=10, command=self.printController.cancelPrint, state="disabled")
        self.cancelPrintButton.grid(row=0, column=3, padx=5, pady=5, sticky=N+S)

        # ==========| Temperatures Frame |==========
        self.temperatureFrame = Frame(self.printTab, highlightthickness=2, highlightbackground="#000000")
        self.temperatureFrame.grid(row=1, column=0, padx=5, pady=5, sticky=W+E+N+S)

        # ==========| Monitoring Frame |==========
        self.printMonitorFrame = Frame(self.printTab, highlightthickness=2, highlightbackground="#000000")
        self.printMonitorFrame.grid(row=1, column=1, padx=5, pady=5, sticky=W+E+N+S)
        # Progress Label
        self.progressLabel = Label(self.printMonitorFrame, text="Progress:")
        self.progressLabel.grid(row=0, column=0, padx=5, pady=5, sticky=W)
        # Progress bar
        self.progressBar = ttk.Progressbar(self.printMonitorFrame, orient=HORIZONTAL, length=400, mode="determinate")
        self.progressBar.grid(row=1, column=0, columnspan=2, padx=5, pady=5)
        self.progressBar['value'] = 100

        self.textScroll = Scrollbar(self.printMonitorFrame, orient="vertical")
        self.textBox = Text(self.printMonitorFrame,
                            wrap=NONE,
                            width=49,
                            height=18,
                            yscrollcommand=self.textScroll.set,
                            state="disabled"
                            )
        self.textScroll.config(command=self.textBox.yview)
        self.textBox.grid(row=2, column=0, columnspan=1, padx=(5,0), pady=5)
        self.textScroll.grid(row=2, column=1, padx=(0, 5), pady=5, sticky=N+S)

    def fillArmTab(self):
        # ==========| Serial Frame |==========
        self.serialFrame = Frame(self.armTab, highlightthickness=2, highlightbackground="#000000")
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
        # Reset button
        self.resetButton = Button(self.serialFrame, text="Reset", command=self.armController.resetAwaitingResponse, width=10)
        self.resetButton.grid(row=3, column=1, padx=5, pady=5)
        # ==========| Reported Position Frame |==========
        self.reportedPosFrame = Frame(self.armTab, highlightthickness=2, highlightbackground="#000000")
        self.reportedPosFrame.grid(row=0, column=1, columnspan=2, padx=5, pady=5, sticky=W+N+E+S)
        # Reported Position label
        self.reportedPosLabel = Label(self.reportedPosFrame, text="Reported Position:")
        self.reportedPosLabel.grid(row=0, column=0, columnspan=5, padx=5, pady=5, sticky=W)
        # Request position button
        self.requestPosButton = Button(self.reportedPosFrame, text="Request Position", command=self.armController.requestPositionManual, width=15)
        self.requestPosButton.grid(row=0, column=1, padx=5, pady=5, sticky=E)
        # Reported position coordinate labels
        # XYZ
        self.xyzPosFrame = Frame(self.reportedPosFrame, highlightthickness=1, highlightbackground="#000000")
        self.xyzPosFrame.grid(row=1, column=0, padx=5, pady=5)
        # Create them
        self.xCurCoordLabel = Label(self.xyzPosFrame, text="X:")
        self.xCurCoord = Label(self.xyzPosFrame, text="xxx") # 'xxx' until value reported
        self.yCurCoordLabel = Label(self.xyzPosFrame, text="Y:")
        self.yCurCoord = Label(self.xyzPosFrame, text="xxx") # 'xxx' until value reported
        self.zCurCoordLabel = Label(self.xyzPosFrame, text="Z:")
        self.zCurCoord = Label(self.xyzPosFrame, text="xxx") # 'xxx' until value reported
        self.RxCurCoordLabel = Label(self.xyzPosFrame, text="Rx:")
        self.RxCurCoord = Label(self.xyzPosFrame, text="xxx") # 'xxx' until value reported
        self.RyCurCoordLabel = Label(self.xyzPosFrame, text="Ry:")
        self.RyCurCoord = Label(self.xyzPosFrame, text="xxx") # 'xxx' until value reported
        self.RzCurCoordLabel = Label(self.xyzPosFrame, text="Rz:")
        self.RzCurCoord = Label(self.xyzPosFrame, text="xxx") # 'xxx' until value reported
        # Display them
        self.xCurCoordLabel.grid(row=0, column=0, padx=5, pady=5)
        self.xCurCoord.grid(row=0, column=1, padx=5, pady=5)
        self.yCurCoordLabel.grid(row=0, column=2, padx=5, pady=5)
        self.yCurCoord.grid(row=0, column=3, padx=5, pady=5)
        self.zCurCoordLabel.grid(row=0, column=4, padx=5, pady=5)
        self.zCurCoord.grid(row=0, column=5, padx=5, pady=5)
        self.RxCurCoordLabel.grid(row=1, column=0, padx=5, pady=5)
        self.RxCurCoord.grid(row=1, column=1, padx=5, pady=5)
        self.RyCurCoordLabel.grid(row=1, column=2, padx=5, pady=5)
        self.RyCurCoord.grid(row=1, column=3, padx=5, pady=5)
        self.RzCurCoordLabel.grid(row=1, column=4, padx=5, pady=5)
        self.RzCurCoord.grid(row=1, column=5, padx=5, pady=5)
        
        # Angles
        self.jointPosFrame = Frame(self.reportedPosFrame, highlightthickness=1, highlightbackground="#000000")
        self.jointPosFrame.grid(row=1, column=1, padx=5, pady=5)
        # Create them
        self.J1CurCoordLabel = Label(self.jointPosFrame, text="J1:")
        self.J1CurCoord = Label(self.jointPosFrame, text="xxx") # 'xxx' until value reported
        self.J2CurCoordLabel = Label(self.jointPosFrame, text="J2:")
        self.J2CurCoord = Label(self.jointPosFrame, text="xxx") # 'xxx' until value reported
        self.J3CurCoordLabel = Label(self.jointPosFrame, text="J3:")
        self.J3CurCoord = Label(self.jointPosFrame, text="xxx") # 'xxx' until value reported
        self.J4CurCoordLabel = Label(self.jointPosFrame, text="J4:")
        self.J4CurCoord = Label(self.jointPosFrame, text="xxx") # 'xxx' until value reported
        self.J5CurCoordLabel = Label(self.jointPosFrame, text="J5:")
        self.J5CurCoord = Label(self.jointPosFrame, text="xxx") # 'xxx' until value reported
        self.J6CurCoordLabel = Label(self.jointPosFrame, text="J6:")
        self.J6CurCoord = Label(self.jointPosFrame, text="xxx") # 'xxx' until value reported
        # Display them
        self.J1CurCoordLabel.grid(row=0, column=0, padx=5, pady=5)
        self.J1CurCoord.grid(row=0, column=1, padx=5, pady=5)
        self.J2CurCoordLabel.grid(row=0, column=2, padx=5, pady=5)
        self.J2CurCoord.grid(row=0, column=3, padx=5, pady=5)
        self.J3CurCoordLabel.grid(row=0, column=4, padx=5, pady=5)
        self.J3CurCoord.grid(row=0, column=5, padx=5, pady=5)
        self.J4CurCoordLabel.grid(row=1, column=0, padx=5, pady=5)
        self.J4CurCoord.grid(row=1, column=1, padx=5, pady=5)
        self.J5CurCoordLabel.grid(row=1, column=2, padx=5, pady=5)
        self.J5CurCoord.grid(row=1, column=3, padx=5, pady=5)
        self.J6CurCoordLabel.grid(row=1, column=4, padx=5, pady=5)
        self.J6CurCoord.grid(row=1, column=5, padx=5, pady=5)

        # ==========| Calibration Frame |==========
        self.calibrationFrame = Frame(self.armTab, highlightthickness=2, highlightbackground="#000000")
        self.calibrationFrame.grid(row=1, column=0, padx=5, pady=5)
        # Calibration label
        self.calibrationLabel = Label(self.calibrationFrame, text="Calibration:")
        self.calibrationLabel.grid(row=0, column=0, padx=5, pady=5, sticky=W+N)
        # Full calibration button
        self.calibrateButton = Button(self.calibrationFrame, text="Calibrate", command=self.armController.startArmCalibration, width=10)
        self.calibrateButton.grid(row=1, column=0, padx=5, pady=5, sticky=W+E)
        # Calibration offsets
        self.calOffsetFrame = Frame(self.calibrationFrame)
        self.calOffsetFrame.grid(row=2, column=0, padx=5, pady=5, sticky=W+E)
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
        # Auto fill a value of '0'
        self.J1OffsetEntry.insert(0, "0")
        self.J2OffsetEntry.insert(0, "0")
        self.J3OffsetEntry.insert(0, "0")
        self.J4OffsetEntry.insert(0, "0")
        self.J5OffsetEntry.insert(0, "0")
        self.J6OffsetEntry.insert(0, "0")

        # Individual calibration buttons
        self.indivCalFrame = Frame(self.calibrationFrame)
        self.indivCalFrame.grid(row=3, column=0, padx=5, pady=5, sticky=W+E)
        # Label
        self.indivCalLabel = Label(self.indivCalFrame, text="Individual Calibrations:")
        self.indivCalLabel.grid(row=0, column=0, columnspan=3, padx=5, pady=5, sticky=W)
        # Make the buttons
        self.calJ1Button = Button(self.indivCalFrame, text="Cal J1", command=lambda: self.armController.calibrateJoints(1, 0, 0, 0, 0, 0), width=7)
        self.calJ2Button = Button(self.indivCalFrame, text="Cal J2", command=lambda: self.armController.calibrateJoints(0, 1, 0, 0, 0, 0), width=7)
        self.calJ3Button = Button(self.indivCalFrame, text="Cal J3", command=lambda: self.armController.calibrateJoints(0, 0, 1, 0, 0, 0), width=7)
        self.calJ4Button = Button(self.indivCalFrame, text="Cal J4", command=lambda: self.armController.calibrateJoints(0, 0, 0, 1, 0, 0), width=7)
        self.calJ5Button = Button(self.indivCalFrame, text="Cal J5", command=lambda: self.armController.calibrateJoints(0, 0, 0, 0, 1, 0), width=7)
        self.calJ6Button = Button(self.indivCalFrame, text="Cal J6", command=lambda: self.armController.calibrateJoints(0, 0, 0, 0, 0, 1), width=7)
        # Place buttons
        self.calJ1Button.grid(row=1, column=0, padx=5, pady=5,)
        self.calJ2Button.grid(row=1, column=1, padx=5, pady=5)
        self.calJ3Button.grid(row=1, column=2, padx=5, pady=5)
        self.calJ4Button.grid(row=2, column=0, padx=5, pady=5)
        self.calJ5Button.grid(row=2, column=1, padx=5, pady=5)
        self.calJ6Button.grid(row=2, column=2, padx=5, pady=5)

        # ==========| Tests Frame |==========
        self.armTestsFrame = Frame(self.armTab, highlightthickness=2, highlightbackground="#000000")
        self.armTestsFrame.grid(row=1, column=1, padx=5, pady=5, sticky=W+E+N+S)
        # Label
        self.armTestsLabel = Label(self.armTestsFrame, text="Arm Tests:")
        self.armTestsLabel.grid(row=0, column=0, padx=5, pady=5, sticky=W+N)

        # Limit switch test
        self.limitTestFrame = Frame(self.armTestsFrame)
        self.limitTestFrame.grid(row=1, column=0, padx=5, pady=5, sticky=W)
        self.limitTestButton = Button(self.limitTestFrame, text="Test Limit Switches", command=self.armController.toggleLimitTest)
        self.limitTestButton.grid(row=0, column=0, columnspan=6, sticky=W)
        # Create the widgets
        self.J1LimLabel = Label(self.limitTestFrame, text="J1:")
        self.J1LimState = Label(self.limitTestFrame, text="x")
        self.J2LimLabel = Label(self.limitTestFrame, text="J2:")
        self.J2LimState = Label(self.limitTestFrame, text="x")
        self.J3LimLabel = Label(self.limitTestFrame, text="J3:")
        self.J3LimState = Label(self.limitTestFrame, text="x")
        self.J4LimLabel = Label(self.limitTestFrame, text="J4:")
        self.J4LimState = Label(self.limitTestFrame, text="x")
        self.J5LimLabel = Label(self.limitTestFrame, text="J5:")
        self.J5LimState = Label(self.limitTestFrame, text="x")
        self.J6LimLabel = Label(self.limitTestFrame, text="J6:")
        self.J6LimState = Label(self.limitTestFrame, text="x")
        # Display the widgets
        self.J1LimLabel.grid(row=1, column=0)
        self.J1LimState.grid(row=1, column=1)
        self.J2LimLabel.grid(row=1, column=2)
        self.J2LimState.grid(row=1, column=3)
        self.J3LimLabel.grid(row=1, column=4)
        self.J3LimState.grid(row=1, column=5)
        self.J4LimLabel.grid(row=2, column=0)
        self.J4LimState.grid(row=2, column=1)
        self.J5LimLabel.grid(row=2, column=2)
        self.J5LimState.grid(row=2, column=3)
        self.J6LimLabel.grid(row=2, column=4)
        self.J6LimState.grid(row=2, column=5)

        # Encoder test
        self.encoderTestFrame = Frame(self.armTestsFrame)
        self.encoderTestFrame.grid(row=2, column=0, padx=5, pady=5, sticky=W)
        self.encoderTestButton = Button(self.encoderTestFrame, text="Test Encoders", command=self.armController.toggleEncoderTest)
        self.encoderTestButton.grid(row=0, column=0, columnspan=6, sticky=W)
        # Create the widgets
        self.J1EncLabel = Label(self.encoderTestFrame, text="J1:")
        self.J1EncState = Label(self.encoderTestFrame, text="xxxx")
        self.J2EncLabel = Label(self.encoderTestFrame, text="J2:")
        self.J2EncState = Label(self.encoderTestFrame, text="xxxx")
        self.J3EncLabel = Label(self.encoderTestFrame, text="J3:")
        self.J3EncState = Label(self.encoderTestFrame, text="xxxx")
        self.J4EncLabel = Label(self.encoderTestFrame, text="J4:")
        self.J4EncState = Label(self.encoderTestFrame, text="xxxx")
        self.J5EncLabel = Label(self.encoderTestFrame, text="J5:")
        self.J5EncState = Label(self.encoderTestFrame, text="xxxx")
        self.J6EncLabel = Label(self.encoderTestFrame, text="J6:")
        self.J6EncState = Label(self.encoderTestFrame, text="xxxx")
        # Display the widgets
        self.J1EncLabel.grid(row=1, column=0)
        self.J1EncState.grid(row=1, column=1)
        self.J2EncLabel.grid(row=1, column=2)
        self.J2EncState.grid(row=1, column=3)
        self.J3EncLabel.grid(row=1, column=4)
        self.J3EncState.grid(row=1, column=5)
        self.J4EncLabel.grid(row=2, column=0)
        self.J4EncState.grid(row=2, column=1)
        self.J5EncLabel.grid(row=2, column=2)
        self.J5EncState.grid(row=2, column=3)
        self.J6EncLabel.grid(row=2, column=4)
        self.J6EncState.grid(row=2, column=5)

        # ==========| Movement Frame |==========
        self.moveFrame = Frame(self.armTab, highlightthickness=2, highlightbackground="#000000")
        self.moveFrame.grid(row=1, column=2, padx=5, pady=5, sticky=W+E+N+S)
        # ===| Linear Move |===
        self.linearMoveFrame = Frame(self.moveFrame, highlightthickness=1, highlightbackground="#000000")
        self.linearMoveFrame.grid(row=0, column=0, padx=5, pady=5, sticky=W+E+N+S)
        self.linearMoveLabel = Label(self.linearMoveFrame, text="Linear Move:")
        self.linearMoveLabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        # Populate xyz button
        self.getXYZButton = Button(self.linearMoveFrame, text="Get XYZ", command = self.armController.populateMJ)
        self.getXYZButton.grid(row=0, column = 2, columnspan=2, padx=5, pady=5)
        # Send command button
        self.linearMoveButton = Button(self.linearMoveFrame, text="Send MJ", command=self.armController.prepMJCommand)
        self.linearMoveButton.grid(row=0, column=4, columnspan=2, padx=5, pady=5, sticky=E)

        # Coordinate labels and text boxes
        # Create them
        self.xCoordLabel = Label(self.linearMoveFrame, text="X:")
        self.xCoordEntry = Entry(self.linearMoveFrame, width=6)
        self.yCoordLabel = Label(self.linearMoveFrame, text="Y:")
        self.yCoordEntry = Entry(self.linearMoveFrame, width=6)
        self.zCoordLabel = Label(self.linearMoveFrame, text="Z:")
        self.zCoordEntry = Entry(self.linearMoveFrame, width=6)
        self.RxCoordLabel = Label(self.linearMoveFrame, text="Rx:")
        self.RxCoordEntry = Entry(self.linearMoveFrame, width=6)
        self.RyCoordLabel = Label(self.linearMoveFrame, text="Ry:")
        self.RyCoordEntry = Entry(self.linearMoveFrame, width=6)
        self.RzCoordLabel = Label(self.linearMoveFrame, text="Rz:")
        self.RzCoordEntry = Entry(self.linearMoveFrame, width=6)
        # Display them
        self.xCoordLabel.grid(row=1, column=0, padx=(5, 0), pady=5)
        self.xCoordEntry.grid(row=1, column=1, padx=(0, 5), pady=5)
        self.yCoordLabel.grid(row=1, column=2, padx=(5, 0), pady=5)
        self.yCoordEntry.grid(row=1, column=3, padx=(0, 5), pady=5)
        self.zCoordLabel.grid(row=1, column=4, padx=(5, 0), pady=5)
        self.zCoordEntry.grid(row=1, column=5, padx=(0, 5), pady=5)
        self.RxCoordLabel.grid(row=2, column=0, padx=(5, 0), pady=5)
        self.RxCoordEntry.grid(row=2, column=1, padx=(0, 5), pady=5)
        self.RyCoordLabel.grid(row=2, column=2, padx=(5, 0), pady=5)
        self.RyCoordEntry.grid(row=2, column=3, padx=(0, 5), pady=5)
        self.RzCoordLabel.grid(row=2, column=4, padx=(5, 0), pady=5)
        self.RzCoordEntry.grid(row=2, column=5, padx=(0, 5), pady=5) 

        # ===| Joint move |===
        self.jointMoveFrame = Frame(self.moveFrame, highlightthickness=1, highlightbackground="#000000")
        self.jointMoveFrame.grid(row=1, column=0, padx=5, pady=5, sticky=W+E+N+S)
        self.jointMoveLabel = Label(self.jointMoveFrame, text="Joint Move:")
        self.jointMoveLabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        # Get Joints
        self.getJointsButton = Button(self.jointMoveFrame, text="Get Joints", command = self.armController.populateJoints)
        self.getJointsButton.grid(row=0, column = 2, columnspan=2, padx=5, pady=5)
        # Send command button
        self.jointMoveButton = Button(self.jointMoveFrame, text="Send RJ", command=self.armController.prepRJCommand)
        self.jointMoveButton.grid(row=0, column=4, columnspan=3, padx=5, pady=5)
        # Joint labels and text boxes
        # Create them
        self.J1CoordLabel = Label(self.jointMoveFrame, text="J1:")
        self.J1CoordEntry = Entry(self.jointMoveFrame, width=6)
        self.J2CoordLabel = Label(self.jointMoveFrame, text="J2:")
        self.J2CoordEntry = Entry(self.jointMoveFrame, width=6)
        self.J3CoordLabel = Label(self.jointMoveFrame, text="J3:")
        self.J3CoordEntry = Entry(self.jointMoveFrame, width=6)
        self.J4CoordLabel = Label(self.jointMoveFrame, text="J4:")
        self.J4CoordEntry = Entry(self.jointMoveFrame, width=6)
        self.J5CoordLabel = Label(self.jointMoveFrame, text="J5:")
        self.J5CoordEntry = Entry(self.jointMoveFrame, width=6)
        self.J6CoordLabel = Label(self.jointMoveFrame, text="J6:")
        self.J6CoordEntry = Entry(self.jointMoveFrame, width=6)
        # Display them
        self.J1CoordLabel.grid(row=1, column=0, padx=(5, 0), pady=5)
        self.J1CoordEntry.grid(row=1, column=1, padx=(0, 5), pady=5)
        self.J2CoordLabel.grid(row=1, column=2, padx=(5, 0), pady=5)
        self.J2CoordEntry.grid(row=1, column=3, padx=(0, 5), pady=5)
        self.J3CoordLabel.grid(row=1, column=4, padx=(5, 0), pady=5)
        self.J3CoordEntry.grid(row=1, column=5, padx=(0, 5), pady=5)
        self.J4CoordLabel.grid(row=2, column=0, padx=(5, 0), pady=5)
        self.J4CoordEntry.grid(row=2, column=1, padx=(0, 5), pady=5)
        self.J5CoordLabel.grid(row=2, column=2, padx=(5, 0), pady=5)
        self.J5CoordEntry.grid(row=2, column=3, padx=(0, 5), pady=5)
        self.J6CoordLabel.grid(row=2, column=4, padx=(5, 0), pady=5)
        self.J6CoordEntry.grid(row=2, column=5, padx=(0, 5), pady=5)
        #Move to safe position button
        self.moveToSafeButton = Button(self.moveFrame, text="Move to Safe Position", command=self.armController.moveSafe, width=20)
        self.moveToSafeButton.grid(row=6, column=0, columnspan=6, padx=5, pady=5)
        self.moveToHomeButton = Button(self.moveFrame, text="Move to Home Position", command=self.armController.moveHome, width=20)
        self.moveToHomeButton.grid(row=7, column=0, columnspan=6, padx=5, pady=5)
        # ==========| Loop Frame |==========
        self.loopFrame = Frame(self.armTab, highlightthickness=2, highlightbackground="#000000")
        self.loopFrame.grid(row=0, column=3, padx=5, pady=5, sticky=W+E+N+S)
        self.loopFrameLabel = Label(self.loopFrame, text="Loop Mode:")
        self.loopFrameLabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        self.openLoopButton = Button(self.loopFrame, text="Open Loop", command=self.armController.setOpenLoop)
        self.openLoopButton.grid(row=1, column=0, padx=5, pady=5)
        self.closedLoopButton = Button(self.loopFrame, text="Closed Loop", command=self.armController.setClosedLoop)
        self.closedLoopButton.grid(row=1, column=1, padx=5, pady=5)
        self.loopStatusLabel = Label(self.loopFrame, text="Status:")
        self.loopStatusLabel.grid(row=2, column=0, padx=5, pady=5, sticky=W)
        self.loopStatus = Label(self.loopFrame, text="Unknown")
        self.loopStatus.grid(row=2, column=1, padx=5, pady=5, sticky=W)

        # ==========| Origin Frame |==========
        self.originFrame = Frame(self.armTab, highlightthickness=2, highlightbackground="#000000")
        self.originFrame.grid(row=1, column=3, padx=5, pady=5, sticky=W+E+N+S)
        self.originFrameLabel = Label(self.originFrame, text="Origin:")
        self.originFrameLabel.grid(row=0, column=0, columnspan=3, padx=5, pady=5, sticky=W)
        # Send command button
        self.setOrigin = Button(self.originFrame, text="Set Origin At Current Position", command=self.armController.setOrigin)
        self.setOrigin.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky=W)

        # Coordinate labels and text boxes
        # Create them
        self.xyzOriginFrame = Frame(self.originFrame, highlightthickness=1, highlightbackground="#000000")
        self.xyzOriginFrame.grid(row=2, column=0, padx=5, pady=5)
        # Create the widgets
        self.xCurCoordOriginLabel = Label(self.xyzOriginFrame, text="X:")
        self.xCurCoordOrigin = Label(self.xyzOriginFrame, text="xxx") # 'xxx' until value reported
        self.yCurCoordOriginLabel = Label(self.xyzOriginFrame, text="Y:")
        self.yCurCoordOrigin = Label(self.xyzOriginFrame, text="xxx") # 'xxx' until value reported
        self.zCurCoordOriginLabel = Label(self.xyzOriginFrame, text="Z:")
        self.zCurCoordOrigin = Label(self.xyzOriginFrame, text="xxx") # 'xxx' until value reported
        # Display the widgets
        self.xCurCoordOriginLabel.grid(row=0, column=0, padx=5, pady=5)
        self.xCurCoordOrigin.grid(row=0, column=1, padx=5, pady=5)
        self.yCurCoordOriginLabel.grid(row=0, column=2, padx=5, pady=5)
        self.yCurCoordOrigin.grid(row=0, column=3, padx=5, pady=5)
        self.zCurCoordOriginLabel.grid(row=0, column=4, padx=5, pady=5)
        self.zCurCoordOrigin.grid(row=0, column=5, padx=5, pady=5)
        #Move to Origin
        self.moveToOrigin = Button(self.originFrame, text="Move To Origin", command=self.armController.moveOrigin)
        self.moveToOrigin.grid(row=3, column=0, columnspan=2, padx=5, pady=5, sticky=W)

        #Delta Origin
        self.deltaOriginLabel = Label(self.originFrame, text="Delta from Origin:")
        self.deltaOriginLabel.grid(row=4, column=0, columnspan=6, padx=5, pady=5, sticky=W)
        
        #Delta coordinates
        self.originDeltaFrame = Frame(self.originFrame, highlightthickness=1, highlightbackground="#000000")
        self.originDeltaFrame.grid(row=5, column=0, padx=5, pady=5)

        self.xDeltaOriginLabel = Label(self.originDeltaFrame, text="ΔX:")
        self.xDeltaOrigin = Label(self.originDeltaFrame, text="xxx") # 'xxx' until value reported
        self.yDeltaOriginLabel = Label(self.originDeltaFrame, text="ΔY:")
        self.yDeltaOrigin = Label(self.originDeltaFrame, text="xxx") # 'xxx' until value reported
        self.zDeltaOriginLabel = Label(self.originDeltaFrame, text="ΔZ:")
        self.zDeltaOrigin = Label(self.originDeltaFrame, text="xxx") # 'xxx' until value reported
        self.xDeltaOriginLabel.grid(row=3, column=0, padx=5, pady=5)
        self.xDeltaOrigin.grid(row=3, column=1, padx=5, pady=5)
        self.yDeltaOriginLabel.grid(row=3, column=2, padx=5, pady=5)
        self.yDeltaOrigin.grid(row=3, column=3, padx=5, pady=5)
        self.zDeltaOriginLabel.grid(row=3, column=4, padx=5, pady=5)
        self.zDeltaOrigin.grid(row=3, column=5, padx=5, pady=5)

    def fillDebugTab(self):
        # ==========| Variables Frame |==========
        self.debugVarFrame = Frame(self.debugTab, highlightthickness=2, highlightbackground="#000000", width=200, height=460)
        self.debugVarFrame.grid(row=0, column=0, padx=5, pady=5)
        self.debugVarFrame.grid_propagate(False)
        # ===| SerialController Variables |===
        self.serDebugFrame = Frame(self.debugVarFrame, highlightthickness=1, highlightbackground="#000000")
        self.serDebugFrame.grid(row=0, column=0, padx=5, pady=5, sticky=W)
        self.serDebugLabel = Label(self.serDebugFrame, text="SerialController:")
        self.serDebugLabel.grid(row=0, column=0, padx=5, pady=5, sticky=W)
        # boardConnected
        self.serDebugBoardLabel = Label(self.serDebugFrame, text="boardConnected = ")
        self.serDebugBoardLabel.grid(row=1, column=0, padx=5, pady=5, sticky=W)
        # waitingForResponse
        self.serDebugWaitLabel = Label(self.serDebugFrame, text="waitingForResponse = ")
        self.serDebugWaitLabel.grid(row=2, column=0, padx=5, pady=5, sticky=W)
        # responseReady
        self.serDebugRespLabel = Label(self.serDebugFrame, text="responseReady = ")
        self.serDebugRespLabel.grid(row=3, column=0, padx=5, pady=5, sticky=W)

        # ===| ArmController Variables |===
        self.armDebugFrame = Frame(self.debugVarFrame, highlightthickness=1, highlightbackground="#000000")
        self.armDebugFrame.grid(row=1, column=0, padx=5, pady=5, sticky=W)
        self.armDebugLabel = Label(self.armDebugFrame, text="ArmController:")
        self.armDebugLabel.grid(row=0, column=0, padx=5, pady=5, sticky=W)
        # armCalibrated
        self.armDebugCalLabel = Label(self.armDebugFrame, text="armCalibrated = ")
        self.armDebugCalLabel.grid(row=1, column=0, padx=5, pady=5, sticky=W)
        # calibrationinProgress
        self.armDebugCalInProgLabel = Label(self.armDebugFrame, text="calibrationInProgress = ")
        self.armDebugCalInProgLabel.grid(row=2, column=0, padx=5, pady=5, sticky=W)
        # calibrationState
        self.armDebugCalStateLabel = Label(self.armDebugFrame, text="calibrationState = ")
        self.armDebugCalStateLabel.grid(row=3, column=0, padx=5, pady=5, sticky=W)
        self.overrideCalibrationButton = Button(self.debugVarFrame,text="Override Calibration", command=self.armController.overrideCalibration)
        self.overrideCalibrationButton.grid(row=3, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        # ==========| Terminal Frame |==========
        # ===| PrintController Variables |===
        
        self.termFrame = Frame(self.debugTab, bg="#00FFFF", highlightthickness=2, highlightbackground="#000000")
        self.termFrame.grid(row=0, column=1, padx=5, pady=5, sticky=W+E+N+S)

        self.termVertScroll = Scrollbar(self.termFrame, orient="vertical")
        self.termHorzScroll = Scrollbar(self.termFrame, orient="horizontal")

        self.terminal = Text(self.termFrame,
                            wrap=NONE,
                            width=65,
                            height=27,
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

    def fillSettingsTab(self):
        # Temporary text to inform user that there is nothing here yet
        Label(self.settingsTab, text="Nothing to see here at the moment (WIP)").pack(fill="both", expand=True)
       
    def update(self):
        self.updateDebugVars() # Update the debug tab variables

        # ==========| SerialController |==========

        # ===========| ArmController |============
        # If arm calibration is in progress, call the calibration update function
        if self.armController.calibrationInProgress:
            self.armController.calibrateArmUpdate()
        if self.armController.awaitingMoveResponse:
            self.armController.moveUpdate()
        if self.armController.testingLimitSwitches:
            self.armController.limitTestUpdate()
        if self.armController.testingEncoders:
            self.armController.encoderTestUpdate()
        if self.armController.awaitingPosResponse:
            self.armController.requestPositionUpdate()
        if self.printController.printing:
            self.printController.printLoop()
        # ==========| PrintController |==========


        # Set up another call to the update function after updateDelay milliseconds
        self.after(self.updateDelay, self.update)

    def updateDebugVars(self):
        # SerialController vars
        self.serDebugBoardLabel.config(text=f"boardConnected = {self.serialController.boardConnected}")
        self.serDebugWaitLabel.config(text=f"waitingForResponse = {self.serialController.waitingForResponse}")
        self.serDebugRespLabel.config(text=f"responseReady = {self.serialController.responseReady}")

        # ArmController vars
        self.armDebugCalLabel.config(text=f"armCalibrated = {self.armController.armCalibrated}")
        self.armDebugCalInProgLabel.config(text=f"calibrationInProgress = {self.armController.calibrationInProgress}")
        self.armDebugCalStateLabel.config(text=f"calibrationState = {self.armController.calibrationState}")

        # PrintController vars

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

    # Used to print to the in window terminal
    def terminalPrint(self, message):
        print(message) # Print to the Python terminal as well
        self.terminal.config(state="normal") # Need to enable to modify
        self.terminal.insert(END, f"{datetime.now().now()}| {message}\n") # Print the message with the current time fixated at the front
        self.terminal.config(state="disabled") # Disable again to avoid user changes
        self.terminal.see("end") # Forces terminal to autoscroll with new text. Probably want to make this a toggle option

    # Used to print important info to the status bar
    def statusPrint(self, message):
        # TODO: Add extra check for if text is too long and do a fancy '...' or something to show that there is more
        # Update the status label
        self.statusLabel.config(text=f"Status: {message}")
        # Also print the full message to the terminal
        self.terminalPrint(message)

if __name__ == "__main__":
    app = TkWindow()
    app.mainloop()