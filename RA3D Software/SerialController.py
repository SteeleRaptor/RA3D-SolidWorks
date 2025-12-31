import serial
import time

# TODO: Needs threading implemented to prevent GUI from freezing while waiting for a response from the arm
# TODO: Could include a timeout but some arm move commands (such as calibration) will take much longer than others

class SerialController:

    def __init__(self, root):
        self.root = root

        self.port = ""
        self.baud = 9600
        self.board = None
        self.boardConnected = False

    def connectPort(self, port):
        self.port = port
        try:
            tempBoard = serial.Serial(self.port, self.baud)
            print(f"Port {port} opened successfully")
            self.board = tempBoard
            self.boardConnected = True
        except Exception:
            print(f"Failed to open port: {port}")

    def disconnectPort(self):
        if self.boardConnected == True:
            self.board.close()

    # Sends a string over serial to the connected port and returns a response from the board
    def sendSerial(self, command):
        if self.boardConnected == False:
            print("Failed to send command: No board connected")
            return None
        print(f"Sending Command: {command}")
        self.board.write(command.encode())
        self.board.reset_input_buffer()
        time.sleep(0.1)
        self.response = str(self.board.readline().strip(), 'utf-8')
        print(f"Response Received: {self.response}")
        return self.response

