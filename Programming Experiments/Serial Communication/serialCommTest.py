
import serial
import time
import os

os.system("cls")

def sendSerial(commandToSend):
    global board
    print(f"Sending Command: {commandToSend}")
    board.write(commandToSend.encode())
    board.reset_input_buffer()
    time.sleep(0.1)
    responseFromBoard = str(board.readline().strip(), 'utf-8')
    print(f"Response Received: {responseFromBoard}")
    return responseFromBoard

def fullCalibration():
    # Need to calibrate first to prevent breaking things
    print("\n=====| Starting Full Calibration|=====\n")
    # Taken from AR4.py, line 8308
    # command = "LL"+"A"+str(CAL['J1CalStatVal'].get())+"B"+str(CAL['J2CalStatVal'].get())+"C"+str(CAL['J3CalStatVal'].get())+"D"+str(CAL['J4CalStatVal'].get())+"E"+str(CAL['J5CalStatVal'].get())+"F"+str(CAL['J6CalStatVal'].get())+"G"+str(CAL['J7CalStatVal'].get())+"H"+str(CAL['J8CalStatVal'].get())+"I"+str(CAL['J9CalStatVal'].get())+"J"+str(CAL['J1calOff'])+"K"+str(CAL['J2calOff'])+"L"+str(CAL['J3calOff'])+"M"+str(CAL['J4calOff'])+"N"+str(CAL['J5calOff'])+"O"+str(CAL['J6calOff'])+"P"+str(CAL['J7calOff'])+"Q"+str(CAL['J8calOff'])+"R"+str(CAL['J9calOff'])+"\n" 
    calJStage1 = [1, 1, 1, 0, 0, 0]
    calJStage2 = [0, 0, 0, 1, 1, 1]
    stage1CalSuccess = False
    stage2CalSuccess = False

    command = f"LLA{calJStage1[0]}B{calJStage1[1]}C{calJStage1[2]}D{calJStage1[3]}E{calJStage1[4]}F{calJStage1[5]}G0H0I0J0K0L0M0N0O0P0Q0\n"
    response = sendSerial(command)

    # Check if Stage 1 calibration was successful
    if (response[:1] == 'A'):
        print("Stage 1 Calibration Successful")
        printPosition(response)
        stage1CalSuccess = True
    else:
        print("Stage 1 Calibration FAILED")
        raise Exception("Stage 2 Calibration FAILED")

    # If Stage 1 calibration was successful, start stage 2 calibration
    if (stage1CalSuccess):
        command = f"LLA{calJStage2[0]}B{calJStage2[1]}C{calJStage2[2]}D{calJStage2[3]}E{calJStage2[4]}F{calJStage2[5]}G0H0I0J0K0L0M0N0O0P0Q0\n"
        response = sendSerial(command)

        if(response[:1] == 'A'):
            print("Stage 2 Calibration Successful")
            stage2CalSuccess = True
            printPosition(response)
        else:
            print("Stage 2 Calibration FAILED")
            raise Exception("Stage 2 Calibration FAILED")
        
    # If either calibration fails, close communication to board and stop the program
    if (not (stage1CalSuccess and stage2CalSuccess)):
        print("Closing connection to board due to calibration failure")
        board.close()
        quit()
        return False
    else:
        print("\n=====| Calibration Successful |=====\n")
        return True

# Move Linear Command
def sendML(X, Y, Z, Rx, Ry, Rz):
    global board
    Sp = 25
    Ac = 15
    Dc = 15
    Rm = 80
    # Taken from AR4.py, line XXXX
    # command = "ML"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"Rnd"+Rounding+"W"+RUN['WC']+"Lm"+LoopMode+"Q"+DisWrist+"\n"
    # Create the command
    command = f"MLX{X}Y{Y}Z{Z}Rz{Rz}Ry{Ry}Rx{Rx}J70.00J80.00J90.00Sp{Sp}Ac{Ac}Dc{Dc}Rm{Rm}Rnd0WFLm000000Q0\n"
    # Send the command
    response = sendSerial(command)
    # Check for error
    if (response[:1] == 'E'):
        raise Exception("Error from ML Command")
    else:
        #print("No Error in ML Command")
        pass
    printPosition(response)

# Rotate Joint Command
def sendRJ(J1, J2, J3, J4, J5, J6, Rx, Ry, Rz):
    global board
    Sp = 25
    Ac = 15
    Dc = 15
    Rm = 80
    # RJA0B0C0D0E0F0J70J80J90Sp25Ac10Dc10Rm80WNLm000000
    command = f"RJA{J1}B{J2}C{J3}D{J4}E{J5}F{J6}J7{0}J8{0}J9{0}Sp{Sp}Ac{Ac}Dc{Dc}Rm{Rm}WNLm000000\n"
    # Send the command
    response = sendSerial(command)
    # Check for error
    if (response[:1] == 'E'):
        raise Exception("Error from RJ Command")
    else:
        print("No Error in RJ Command")
    printPosition(response)

def printPosition(receivedResponse):
    # Collect all the indexes for finding values
    # General formatting of a response: A[val]B[val]C[val]D[val]E[val]F[val]G[val]H[val]I[val]J[val]K[val]L[val]M[val]N[val]O[val]P[val]Q[val]R[val]
    J1Idx = receivedResponse.find('A') # A value is angle of J1
    J2Idx = receivedResponse.find('B') # B value is angle of J2
    J3Idx = receivedResponse.find('C') # C value is angle of J3
    J4Idx = receivedResponse.find('D') # D value is angle of J4
    J5Idx = receivedResponse.find('E') # E value is angle of J5
    J6Idx = receivedResponse.find('F') # F value is angle of J6
    XPosIdx = receivedResponse.find('G') # G value is X position
    YPosIdx = receivedResponse.find('H')
    ZPosIdx = receivedResponse.find('I')
    RzIdx = receivedResponse.find('J')
    RyIdx = receivedResponse.find('K')
    RxIdx = receivedResponse.find('L')
    SpeedViolationIdx = receivedResponse.find('M')
    DebugIdx = receivedResponse.find('N')
    FlagIdx = receivedResponse.find('O')
    J7Idx = receivedResponse.find('P')
    J8Idx = receivedResponse.find('Q')
    J9Idx = receivedResponse.find('R')

    # Extract the actual values from the response
    # Joint angles
    J1Angle = receivedResponse[J1Idx+1:J2Idx].strip()
    J2Angle = receivedResponse[J2Idx+1:J3Idx].strip()
    J3Angle = receivedResponse[J3Idx+1:J4Idx].strip()
    J4Angle = receivedResponse[J4Idx+1:J5Idx].strip()
    J5Angle = receivedResponse[J5Idx+1:J6Idx].strip()
    J6Angle = receivedResponse[J6Idx+1:XPosIdx].strip()

    # XYZ Positions
    xPos = receivedResponse[XPosIdx+1:YPosIdx].strip()
    yPos = receivedResponse[YPosIdx+1:ZPosIdx].strip()
    zPos = receivedResponse[ZPosIdx+1:RzIdx].strip()

    # RXYZ Angles
    RxAngle = receivedResponse[RxIdx+1:SpeedViolationIdx].strip()
    RyAngle = receivedResponse[RyIdx+1:RxIdx].strip()
    RzAngle = receivedResponse[RzIdx+1:RyIdx].strip()

    # Print info
    print(f"J1: {J1Angle}, J2: {J2Angle}, J3: {J3Angle}, J4: {J4Angle}, J5: {J5Angle}, J6: {J6Angle}")
    print(f"X: {xPos}, Y: {yPos}, Z:{zPos}")
    print(f"Rx: {RxAngle}, Ry: {RyAngle}, Rz: {RzAngle}")


port = "COM4"
baud = 9600

board = serial.Serial(port, baud)

fullCalibration()

# Send a move command

print("ML: X400, Y0, Z400, Rx0, Ry10, Rz0")
sendML(400, 0, 400, 0, 90, 0)


print("\n=====| End of Program |=====\n")

board.close()
print("\n=====| Communication w/ Board Closed |=====")