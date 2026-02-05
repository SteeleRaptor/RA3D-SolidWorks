%
G90                 ; Absolute positioning
G21                 ; Units in mm

; --- Move to start position ---
G1 X0 Y0 Z0 F1000

; --- Begin infinite loop ---

G1 X100 Y0 Z0 F1000  ; Move to corner 1
G1 X100 Y100 Z0     ; Move to corner 2
G1 X0 Y100 Z0       ; Move to corner 3
G1 X0 Y0 Z0         ; Return to start (corner 4)

G4 P1               ; 1 second pause at corner (optional)

; --- End program ---
M30
%
