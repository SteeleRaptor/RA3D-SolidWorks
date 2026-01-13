
from SerialController import SerialController

class ArmController:
    def __init__(self, root, serialController):
        # Save references to the main window and the serial controller
        self.root = root
        self.serialController = serialController

        self.armCalibrated = False

        self.speed = 25
        self.acceleration = 15
        self.deceleration = 15
        self.ramp = 80

        self.curJ1 = None
        self.curJ2 = None
        self.curJ3 = None
        self.curJ4 = None
        self.curJ5 = None
        self.curJ6 = None
        self.curX = None
        self.curY = None
        self.curZ = None
        self.curRx = None
        self.curRy = None
        self.curRz = None

        self.J1CalOffset = 0
        self.J2CalOffset = 0
        self.J3CalOffset = 0
        self.J4CalOffset = 0
        self.J5CalOffset = 0
        self.J6CalOffset = 0
        

    def calibrateArm(self):
        print("\n=====| Starting Full Calibration |=====\n")
        if self.serialController.boardConnected is False:
            print("Error: No board connected, exiting calibration")
            return
        # Taken from AR4.py, line 8308
        # command = "LL"+"A"+str(CAL['J1CalStatVal'].get())+"B"+str(CAL['J2CalStatVal'].get())+"C"+str(CAL['J3CalStatVal'].get())+"D"+str(CAL['J4CalStatVal'].get())+"E"+str(CAL['J5CalStatVal'].get())+"F"+str(CAL['J6CalStatVal'].get())+"G"+str(CAL['J7CalStatVal'].get())+"H"+str(CAL['J8CalStatVal'].get())+"I"+str(CAL['J9CalStatVal'].get())+"J"+str(CAL['J1calOff'])+"K"+str(CAL['J2calOff'])+"L"+str(CAL['J3calOff'])+"M"+str(CAL['J4calOff'])+"N"+str(CAL['J5calOff'])+"O"+str(CAL['J6calOff'])+"P"+str(CAL['J7calOff'])+"Q"+str(CAL['J8calOff'])+"R"+str(CAL['J9calOff'])+"\n" 
        calJStage1 = [1, 1, 1, 0, 0, 0]
        calJStage2 = [0, 0, 0, 1, 1, 1]
        stage1CalSuccess = False
        stage2CalSuccess = False

        response = self.calibrateJoints(calJStage1[0], calJStage1[1], calJStage1[2], calJStage1[3], calJStage1[4], calJStage1[5])

        # command = f"LLA{calJStage1[0]}B{calJStage1[1]}C{calJStage1[2]}D{calJStage1[3]}E{calJStage1[4]}F{calJStage1[5]}G0H0I0J0K0L0M0N0O0P0Q0\n"
        # response = self.serialController.sendSerial(command)

        # Check if Stage 1 calibration was successful
        if (response[:1] == 'A'):
            print("Stage 1 Calibration Successful")
            self.processPosition(response)
            stage1CalSuccess = True
        else:
            print("Stage 1 Calibration FAILED")
            # Turn on the warning lights on the UI or something
        
        # If Stage 1 calibration successful, start stage 2 calibration
        if (stage1CalSuccess):
            # command = f"LLA{calJStage2[0]}B{calJStage2[1]}C{calJStage2[2]}D{calJStage2[3]}E{calJStage2[4]}F{calJStage2[5]}G0H0I0J0K0L0M0N0O0P0Q0\n"
            # response = self.serialController.sendSerial(command)
            response = self.calibrateJoints(calJStage2[0], calJStage2[1], calJStage2[2], calJStage2[3], calJStage2[4], calJStage2[5])

            if (response[:1] == 'A'):
                print("Stage 2 Calibration Successful")
                self.processPosition(response)
                stage2CalSuccess = True
            else:
                print("Stage 2 Calibration FAILED")
                # Turn on the warning lights on the UI or something

        if (not (stage1CalSuccess and stage2CalSuccess)):
            print("Error during calibration")
            self.armCalibrated = False # Keep armCalibrated flag False
            # Could force a port disconnect here as an extra measure of protecting the arm
            return False
        else:
            print("Arm Calibrated Successfully")
            self.armCalibrated = True
            return True
        
    def calibrateJoints(self, calJ1=False, calJ2=False, calJ3=False, calJ4=False, calJ5=False, calJ6=False):
        if self.serialController.boardConnected is False:
            print("Error: No board connected, exiting calibration")
            return

        command = f"LLA{calJ1}B{calJ2}C{calJ3}D{calJ4}E{calJ5}F{calJ6}G0H0I0J{self.J1CalOffset}K{self.J2CalOffset}L{self.J3CalOffset}M{self.J4CalOffset}N{self.J5CalOffset}O{self.J6CalOffset}P0Q0\n"
        response = self.serialController.sendSerial(command)

        # Check if calibration was a success
        if (response[:1] == 'A'):
            print("Joint(s) Calibrated Successfully")
        else:
            print("Joint(s) Calibration FAILED")
            # Turn on the warning lights on the UI or something
        return response

    def processPosition(self, response):
        # Collect all the indexes for finding values
        # General formatting of a response: A[val]B[val]C[val]D[val]E[val]F[val]G[val]H[val]I[val]J[val]K[val]L[val]M[val]N[val]O[val]P[val]Q[val]R[val]
        J1Idx = response.find('A') # A value is angle of J1
        J2Idx = response.find('B') # B value is angle of J2
        J3Idx = response.find('C') # C value is angle of J3
        J4Idx = response.find('D') # D value is angle of J4
        J5Idx = response.find('E') # E value is angle of J5
        J6Idx = response.find('F') # F value is angle of J6
        XPosIdx = response.find('G') # G value is X position
        YPosIdx = response.find('H') # H value is Y position
        ZPosIdx = response.find('I') # I value is Z position
        RzIdx = response.find('J') # J value is Rz rotation
        RyIdx = response.find('K') # K value is Ry rotation
        RxIdx = response.find('L') # L value is Rx rotation
        SpeedViolationIdx = response.find('M') # M value is if there is a speed violation
        DebugIdx = response.find('N') # N value is ???
        FlagIdx = response.find('O') # O value is ???
        J7Idx = response.find('P') # P value is angle of J7
        J8Idx = response.find('Q') # Q value is angle of J8
        J9Idx = response.find('R') # R value is angle of J9

        # Extract the actual values from the response
        # Joint angles
        self.curJ1 = response[J1Idx+1:J2Idx].strip()
        self.curJ2 = response[J2Idx+1:J3Idx].strip()
        self.curJ3 = response[J3Idx+1:J4Idx].strip()
        self.curJ4 = response[J4Idx+1:J5Idx].strip()
        self.curJ5 = response[J5Idx+1:J6Idx].strip()
        self.curJ6 = response[J6Idx+1:XPosIdx].strip()

        # XYZ Positions
        self.curX = response[XPosIdx+1:YPosIdx].strip()
        self.curY = response[YPosIdx+1:ZPosIdx].strip()
        self.curZ = response[ZPosIdx+1:RzIdx].strip()

        # RXYZ Angles
        self.curRx = response[RxIdx+1:SpeedViolationIdx].strip()
        self.curRy = response[RyIdx+1:RxIdx].strip()
        self.curRz = response[RzIdx+1:RyIdx].strip()

        # Display values on UI
        self.root.xCurCoord.config(text=self.curX)
        self.root.yCurCoord.config(text=self.curY)
        self.root.zCurCoord.config(text=self.curZ)
        self.root.RxCurCoord.config(text=self.curRx)
        self.root.RyCurCoord.config(text=self.curRy)
        self.root.RzCurCoord.config(text=self.curRz)

    def sendML(self, X, Y, Z, Rx, Ry, Rz):
        print("Sending ML command...")
        self.root.terminalPrint("Sending ML command...")
        # Taken from AR4.py, line XXXX
        # command = "ML"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"Rnd"+Rounding+"W"+RUN['WC']+"Lm"+LoopMode+"Q"+DisWrist+"\n"
        # Create the command
        command = f"MLX{X}Y{Y}Z{Z}Rz{Rz}Ry{Ry}Rx{Rx}J70.00J80.00J90.00Sp{self.speed}Ac{self.acceleration}Dc{self.deceleration}Rm{self.ramp}Rnd0WFLm000000Q0\n"
        self.root.terminalPrint("Command to send:")
        self.root.terminalPrint(command)
        # Check if board is not connected or arm is not calibrated
        if self.serialController.boardConnected is False:
            # Inform user in terminal then quit function to avoid sending instruction
            self.root.terminalPrint("Command not sent due to no board connected")
            return
        elif self.armCalibrated is False:
            # Inform user in terminal then quit function to avoid sending instruction
            self.root.terminalPrint("Command not sent due to arm not calibrated")
            return
        
        # Send the serial command
        response = self.serialController.sendSerial(command)

        # Check for any errors
        if (response[:1] == 'E'):
            self.root.terminalPrint("Error executing ML command")
            print("Error executing ML command")
            # Sound the alarms on UI or something
        else:
            self.root.terminalPrint("ML command executed successfully")
            print("No error executing ML command")
        self.processPosition(response)

    def sendRJ(self, J1, J2, J3, J4, J5, J6):
        print("Sending RJ command...")
        if self.serialController.boardConnected is False:
            print("Error: No board connected, cancelling RJ command")
            return
        # RJA0B0C0D0E0F0J70J80J90Sp25Ac10Dc10Rm80WNLm000000
        command = f"RJA{J1}B{J2}C{J3}D{J4}E{J5}F{J6}J7{0}J8{0}J9{0}Sp{self.speed}Ac{self.acceleration}Dc{self.deceleration}Rm{self.ramp}WNLm000000\n"
        # Send the command
        response = self.serialController.sendSerial(command)
        # Check for any errors
        if (response[:1] == 'E'):
            print("Error executing MJ command")
            # Sound the alarms on UI or something
        else:
            print("No error executing MJ command")
        self.processPosition(response)

    def getCalOffsets(self):
        # Grab values from the entry fields, convert to integers, and save
        self.J1CalOffset = int(self.root.J1OffsetEntry.get())
        self.J2CalOffset = int(self.root.J2OffsetEntry.get())
        self.J3CalOffset = int(self.root.J3OffsetEntry.get())
        self.J4CalOffset = int(self.root.J4OffsetEntry.get())
        self.J5CalOffset = int(self.root.J5OffsetEntry.get())
        self.J6CalOffset = int(self.root.J6OffsetEntry.get())