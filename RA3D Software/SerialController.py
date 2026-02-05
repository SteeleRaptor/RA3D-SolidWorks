import serial
import serial.tools.list_ports
from serial import SerialException
import time
import threading
import queue

# TODO: Interesting bug that occurs but when connecting, disconnecting, then reconnecting, the entire serial buffer gets messed up resulting in everything being read in as junk. Not sure how it is happening nor how to fix.

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
        # Check if the serial controller is already connected to a board
        if self.boardConnected == False:
            # If not, connect to the port currently selected in the port dropdown
            self.root.statusPrint("Attempting port connection")
            self.connectPort(self.root.portSelection.get())
            self.root.statusPrint(f"Connection Status: {self.boardConnected}")

            # Perform a check to see if board was actually connected to
            if self.boardConnected:
                self.root.connectButton.config(text="Disconnect") # Change button text
                self.root.portDropdown.config(state="disabled") # Disable port dropdown
                self.root.refreshCOMButton.config(state="disabled") # Disable refresh button
                self.root.portStatusLabel.config(text="Status: Connected") # Change port status text
        else:
            # If so, disconnect from the port
            self.root.statusPrint("Disconnecting from port")
            self.disconnectPort()
            self.root.statusPrint(f"Connection Status: {self.boardConnected}")
            self.root.connectButton.config(text="Connect") # Change button text
            self.root.portDropdown.config(state="readonly") # Enable port dropdown
            self.root.refreshCOMButton.config(state="normal") # Enable refresh button
            self.root.portStatusLabel.config(text="Status: Disconnected") # Change port status text

    def connectPort(self, port):
        self.port = port
        try:
            ### TODO: Edge case not caught by this try is if connecting to port, 
            # unplug & replug board on same port, disconnect, 
            # then connecting again doesn't throw error even though it 
            # doesn't actually connect to the port and essentially bricks 
            # the program from connecting to another port until restarted
            tempBoard = serial.Serial(self.port, self.baud)
            self.root.statusPrint(f"Port {port} opened successfully")
            self.board = tempBoard
            self.board.reset_output_buffer()
            self.board.reset_input_buffer()
            self.boardConnected = True
            self.responseQueue = queue.Queue()
            self.serialThread = threading.Thread(target=self.serialReader, daemon=True)
            self.serialThread.start()
            
        except SerialException:
            self.root.statusPrint(f"Failed to open port: {port}")

    def disconnectPort(self):
        if self.boardConnected == True:
            self.boardConnected = False
            self.board.reset_output_buffer()
            self.board.reset_input_buffer()
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
            else:
                time.sleep(0.05)
            self.checkResponseQueue()
    
    def checkResponseQueue(self):
        # Check if there is anything in the response queue
        try:
            while True:
                if self.boardConnected:
                    self.lastResponse = self.responseQueue.get_nowait()
                    self.responseReady = True
                    self.waitingForResponse = False
                else:
                    break
        except queue.Empty:
            pass
    #Function process response because correct response is not guaranteed for a command
    def sortResponse(self, response):
        self.root.terminalPrint(f"Received Response: {response}")
        if response[0:1]== "ER":
            self.root.statusPrint(f"Kinematic Error: {response[2:]}")
            self.root.terminalPrint(f"Kinematic Error: {response[2:]}")
        elif response[0:1] == "EL":
            self.root.statusPrint(f"Error Axis Fault: {response[2:]}")
            self.root.terminalPrint(f"Error Axis Fault: {response[2:]}")
        elif response[0:2] == "TL":
            #Limit switch test
            pass
        elif response[0:2] == "RE":
            #read encoders
            pass
        elif response[0:4] == "Done":
            #Home position finished
            #Command set output on/off finished
            #send position to arm
            pass
        elif response[0:2] == "WTDone":
            self.root.statusPrint("Wait command finished")
        elif response[0:2] == "echo":
            self.root.statusPrint(f"Echo: {response[4:]}")
        elif response[0] == "\n":
            pass
        elif response == "TurnHazardMoveStopped":
            self.root.statusPrint(f"Encountered Hazard Move Stopped: {response[2:]}")
        elif response[0:2] == "POS":
            self.root.statusPrint("Position received from arm")
        else:
            self.root.statusPrint(f"Received Unrecognized Response: {response}")
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
            self.root.statusPrint("Failed to send command: No board connected")
            return
        # If a board is connected, check if we are already waiting for a serial response
        if self.waitingForResponse:
            self.root.statusPrint("Failed to send command. Currently awaiting serial response")
            return
        self.board.reset_input_buffer()
        # If we aren't awaiting for a serial response, then send the command
        self.root.statusPrint(f"Sending Command: {command}")
        self.board.write(command.encode())
        self.root.terminalPrint("Command sent")
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
            self.root.statusPrint("No serial ports w/ connected devices found")
        else:
            self.root.statusPrint(f"Found {len(ports)} connected device(s):")
            for port in ports:
                self.root.terminalPrint(f"Name: {port.device}\nDescription: {port.description}\nID: {port.hwid}\n\n")
                returnList.append(port.device)
        return returnList
