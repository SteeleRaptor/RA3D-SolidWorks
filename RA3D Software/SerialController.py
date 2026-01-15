import serial
import serial.tools.list_ports
import time
import threading
import queue

# TODO: Needs threading implemented to prevent GUI from freezing while waiting for a response from the arm
# TODO: Could include a timeout but some arm move commands (such as calibration) will take much longer than others

class SerialController:

    def __init__(self, root):
        self.root = root

        # General Connection Variables
        self.port = ""
        self.baud = 9600
        self.board = None
        self.boardConnected = False

        # Threading Variables
        self.serialThread = None
        self.waitingForResponse = False # Used as a flag to signify if we are already awaiting a response
        self.responseQueue = queue.Queue()
        self.responseReady = False # Used as a flag to check if a response is ready to be read
        self.lastResponse = None # Stores whatever the last response was from serial until requested

    # Handles the "Connect/Disconnect" button being pressed to connect or disconnect the port
    def serialConnect(self):
        self.root.terminalPrint("serialConnect()")
        # Check if the serial controller is already connected to a board
        if self.boardConnected == False:
            # If not, connect to the port currently selected in the port dropdown
            self.root.terminalPrint("Attempting port connection")
            self.connectPort(self.root.portSelection.get())
            self.root.terminalPrint(f"Connection Status: {self.boardConnected}")
            print(f"Connection Status: {self.boardConnected}")
            # Perform a check to see if board was actually connected to
            if self.boardConnected:
                self.root.connectButton.config(text="Disconnect") # Change button text
                self.root.portDropdown.config(state="disabled") # Disable port dropdown
                self.root.refreshCOMButton.config(state="disabled") # Disable refresh button
                self.root.portStatusLabel.config(text="Status: Connected") # Change port status text
        else:
            # If so, disconnect from the port
            self.root.terminalPrint("Disconnecting from port")
            self.disconnectPort()
            print(f"Connection Status: {self.boardConnected}")
            self.root.terminalPrint(f"Connection Status: {self.boardConnected}")
            self.root.connectButton.config(text="Connect") # Change button text
            self.root.portDropdown.config(state="readonly") # Enable port dropdown
            self.root.refreshCOMButton.config(state="normal") # Enable refresh button
            self.root.portStatusLabel.config(text="Status: Disconnected") # Change port status text

    def connectPort(self, port):
        self.port = port
        print(self.port)
        try:
            ### TODO: Edge case not caught by this try is if connecting to port, 
            # unplug & replug board on same port, disconnect, 
            # then connecting again doesn't throw error even though it 
            # doesn't actually connect to the port and essentially bricks 
            # the program from connecting to another port until restarted
            tempBoard = serial.Serial(self.port, self.baud)
            print(f"Port {port} opened successfully")
            self.board = tempBoard
            self.boardConnected = True
            self.serialThread = threading.Thread(target=self.serialReader, daemon=True)
        except Exception:
            print(f"Failed to open port: {port}")

    def disconnectPort(self):
        if self.boardConnected == True:
            self.board.close()

    # Repeatedly checks the serial port for new responses
    def serialReader(self):
        while True:
            # Check if a board is connected
            if self.boardConnected is True:
                # If there are any bytes of data waiting...
                if self.board.in_waiting > 0:
                    # Read them in and store them in the response queue
                    data = str(self.board.readline().decode('utf-8').strip())
                    self.responseQueue.put(data)
    
    def checkResponseQueue(self):
        # Check if there is anything in the response queue
        try:
            while True:
                self.lastResponse = self.responseQueue.get_nowait()
                self.responseReady = True
                self.waitingForResponse = False
        except self.responseQueue.empty:
            pass
        self.root.after(100, self.checkResponseQueue)

    # Returns the last response received
    def getLastResponse(self):
        response = self.lastResponse
        self.responseReady = False
        self.lastResponse = None
        return response
        
    # Sends a string over serial to the connected port
    def sendSerial(self, command):
        # Check if a board is connected
        if self.boardConnected == False:
            print("Failed to send command: No board connected")
            return
        # If a board is connected, check if we are already waiting for a serial response
        if self.waitingForResponse:
            print("Failed to send command. Currently awaiting serial response")
            return
        # If we aren't awaiting for a serial response, then send the command
        print(f"Sending Command: {command}")
        self.board.write(command.encode())
        # Reset the input buffer
        self.board.reset_input_buffer()
        self.waitingForResponse = True # Note that it is always assume that there will be a response to be read back in

    def refreshCOMPorts(self):
        self.root.portList = self.getCOMPorts() # Create options list from found COM ports
        self.root.portSelection.set("Select Port")
        self.root.portDropdown["values"] = self.root.portList
        self.root.portDropdown["textvariable"] = self.root.portSelection

    # Returns a list of all COM ports with a device connected
    def getCOMPorts(self):
        ports = list(serial.tools.list_ports.comports())
        returnList = []
        if not ports:
            self.root.terminalPrint("No serial ports w/ connected devices found")
            print("No serial ports w/ connected devices found")
        else:
            self.root.terminalPrint(f"Found {len(ports)} connected device(s):")
            print(f"Found {len(ports)} connected device(s):\n")
            for port in ports:
                self.root.terminalPrint(f"Name: {port.device}\nDescription: {port.description}\nID: {port.hwid}\n\n")
                print(f"Name: {port.device}\nDescription: {port.description}\nID: {port.hwid}\n\n")
                returnList.append(port.device)
        return returnList
