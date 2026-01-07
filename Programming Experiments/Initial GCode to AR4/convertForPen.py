import re

print("\n=====| Program Running |=====\n")

with open("circle2.gcode", "r") as file:
    lines = file.readlines()

print(f"Total Lines: {len(lines)}")

# Process one line to start

curNewLine = 0
newLines = list()

xBounds = [300, 500]
yBounds = [-100, 100]
zBounds = [100, 300]
#xyCenter = [(xBounds[0] + xBounds[1]) / 2, (yBounds[0] + yBounds[1]) / 2]
Ry = 83
xStart = 484
yStart = -16
zStart = 65
lastX = xBounds[0]
lastY = yBounds[1]
lastZ = zBounds[0]
lastF = 0.0
lastE = 0.0

Sp = 100
Ac = 25
Dc = 25
Rm = 25


for i in range(0, len(lines) - 1):
    line = lines[i]
    # Look for lines that aren't comments
    if line[0] == ';':
        newLines.append(f"##{line[1:]}")
    elif line == "\n":
        newLines.append(line)
    elif line[0:3] == "G28": # Home the printer
        # Home the printer
        x = xBounds[0]
        y = yBounds[0]
        z = zBounds[0]
        lastX = x
        lastY = y
        lastZ = z
        lastF = 0.0
        lastE = 0.0

        newLines.append("## Home the printer\n")
        newLine = f"Move L [Home] X {x} Y {y} Z {z} Rz 0 Ry 90 Rx 0.000 J7 0.00 J8 0.00 J9 0.00 Sp {Sp} Ac {Ac} Dc {Dc} Rm {Rm} $ F\n"
        newLines.append(newLine)

    elif line[0:3] == "G90": # Absolute positioning
        # This would be something for the converter software
        newLines.append(f"##{line}")
    elif line[0:2] == "G0" or line[0:2] == "G1": # Temporary G0 and G1 are equal
        # Absolutely had ChatGPT write this because I don't know Regex
        xMatch = re.search(r"X(-?\d+\.?\d*)", line)
        yMatch = re.search(r"Y(-?\d+\.?\d*)", line)
        zMatch = re.search(r"Z(-?\d+\.?\d*)", line)
        fMatch = re.search(r"F(-?\d+\.?\d*)", line)
        eMatch = re.search(r"E(-?\d+\.?\d*)", line)

        x = float(xMatch.group(1)) if xMatch else None
        y = float(yMatch.group(1)) if yMatch else None
        z = float(zMatch.group(1)) if zMatch else None
        f = float(fMatch.group(1)) if fMatch else None
        e = float(eMatch.group(1)) if eMatch else None

        #print(f"From Command: X: {x}, Y:{y}, Z:{z}, F:{f}, E:{e}")

        # If GCode instruction didn't contain a parameter, pull from last saved value
        # If instruction DID contain a parameter, offset the value to put it in the build volume
        if x == None:
            x = lastX
        else:
            x += xStart
            lastX = x
        if y == None:
            y = lastY
        else:
            y += yStart
            lastY = y
        if z == None:
            z = lastZ
        else:
            z += zStart
            lastZ = z
        if f == None:
            f = lastF
        else:
            lastF = f
        if e == None:
            e = lastE
        else:
            lastE = e

        #print(f"Processed: X: {x}, Y:{y}, Z:{z}, F:{f}, E:{e}")

        # TODO: Additional processing to ensure X, Y, and Z are within build volume
        # TODO: Additional processing for F to control speed or something I guess
        # Note that F is in units per minute (per LinuxCNC specifications)
        # https://linuxcnc.org/docs/html/gcode/machining-center.html#sub:feed-rate

        newLine = f"Move L [*] X {x} Y {y} Z {z} Rz 0 {Ry} Rx 0.000 J7 0.00 J8 0.00 J9 0.00 Sp {Sp} Ac {Ac} Dc {Dc} Rm {Rm} $ F\n"
        newLines.append(newLine)

# Add extra line to move the arm out of the way at the end of the "print"
newLine = f"Move L [*] X {xBounds[0]} Y {yBounds[0]} Z {zBounds[1]} Rz 0 Ry 90 Rx 0.000 J7 0.00 J8 0.00 J9 0.00 Sp {Sp} Ac {Ac} Dc {Dc} Rm {Rm} $ F\n"
newLines.append(newLine)

print("=====| Done Processing |=====")

print("\n")
#print(line)
#print(newLine)

with open("output.AR4", "w") as file:
    for item in newLines:
        file.write(f"{item}")

print("=====| Output Created |=====")
