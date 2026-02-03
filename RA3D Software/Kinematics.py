"""
Inverse Kinematics Module for RA3D Robot Arm
Provides functions to calculate joint angles from desired end-effector positions
and orientations, as well as path planning capabilities.
"""

import numpy as np
import math
import scipy.optimize as optimize

class Kinematics:
    # Initialize the Kinematics class with robot parameters
    def __init__(self, root):
        # Store reference to root window (GUI)
        self.root = root
        # Define the number of degrees of freedom for the robot (6-DOF articulated arm)
        self.Robot_nDOFs = 6
        # Initialize robot kinematic parameter array (DH parameters, tool frame, limits)
        self.Robot_Data = [0.0]*66
        # Matrix to store multiple inverse kinematics solutions (up to 4 solutions per joint)
        self.SolutionMatrix = np.zeros((self.Robot_nDOFs, 4))
        # Current joint angles in degrees
        self.currentJointAngles = np.zeros(6)

        # Current end-effector position and orientation [X, Y, Z, Rx, Ry, Rz]
        self.currentxyzuvw = [0.0]*6
        # Output end-effector position and orientation after motion
        self.outgoingxyzuvw = [0.0]*6
        # Test inverse kinematics calculation for position [300, 0, 200] with rotation [0, 90, 0]
        self.SolveInverseKinematics([300, 0, 200, 0, 90, 0])
        # Estimated joint angles used as initial guess for IK solver
        self.joints_estimate = [None]*self.Robot_nDOFs
        
        # Joint angle limits in degrees (negative and positive limits)
        self.J1Limit = [-170, 170]  # Joint 1 limits
        self.J2Limit = [-42, 90]    # Joint 2 limits
        self.J3Limit = [-89, 52]    # Joint 3 limits
        self.J4Limit = [-165, 165]  # Joint 4 limits
        self.J5Limit = [-105, 105]  # Joint 5 limits
        self.J6Limit = [-155, 155]  # Joint 6 limits
        # Identity matrix representing robot base frame (no offset initially)
        self.Robot_BaseFrame = np.identity(4)

    def verifyIfMoveIsSafe(self, joints):
        # Placeholder method for safety verification of joint configuration
        # Check if proposed joint angles avoid collisions and obstacles
        for i in range(6):
            pass

    def linearPathPlanning(self, xyzuvw_Start, xyzuvw_End, step_mm):
        # Placeholder for linear path planning implementation
        # Calculate waypoints along a straight line in Cartesian space
        pass

    def distanceFromLimitSwitches(self, uvw):
        penalty = 0
        w = 1  # Weight for penalty term
        # Calculate joint angles from current position with new orientation
        jointAngles = self.solveInverseKinematics([self.outgoingxyzuvw[0], self.outgoingxyzuvw[1], self.outgoingxyzuvw[2], uvw[0], uvw[1], uvw[2]])
        # Calculate distance from each joint to its negative limit
        z2 = min(abs(jointAngles[1] - self.J2Limit[0]), abs(jointAngles[1] - self.J2Limit[1]))
        # Distance from J3 to its limits
        z3 = min(abs(jointAngles[2] - self.J3Limit[0]), abs(jointAngles[2] - self.J3Limit[1]))
        # Distance from J4 to its limits
        z4 = min(abs(jointAngles[3] - self.J4Limit[0]), abs(jointAngles[3] - self.J4Limit[1]))
        # Distance from J5 to its limits
        z5 = min(abs(jointAngles[4] - self.J5Limit[0]), abs(jointAngles[4] - self.J5Limit[1]))
        # Distance from J6 to its limits
        z6 = min(abs(jointAngles[5] - self.J6Limit[0]), abs(jointAngles[5] - self.J6Limit[1]))
        # Sum of all distances from limits (metric for optimization)
        ztotal = z2 + z3 + z4 + z5 + z6
        
        # Calculate angular differences for each joint
        delta1 = jointAngles[0] - self.currentJointAngles[0]
        # Change in J2 angle
        delta2 = jointAngles[1] - self.currentJointAngles[1]
        # Change in J3 angle
        delta3 = jointAngles[2] - self.currentJointAngles[2]
        # Change in J4 angle
        delta4 = jointAngles[3] - self.currentJointAngles[3]
        # Change in J5 angle
        delta5 = jointAngles[4] - self.currentJointAngles[4]
        # Change in J6 angle
        delta6 = jointAngles[5] - self.currentJointAngles[5]
        # Penalize large joint movements (greater than 180 degrees)
        if abs(delta1) > 180 or abs(delta2) > 180 or abs(delta3) > 180 or abs(delta4) > 180 or abs(delta5) > 180 or abs(delta6) > 180:
            penalty = penalty + 1000  # Add penalty for large movements
        # Return total distance metric
        return ztotal + w*penalty

    # Iterates uvw (orientation) with fixed xyz to find solution furthest from joint limits
    def iterate_uvw(self, uRange, vRange, wRange):
        # Define optimization function to minimize distance to joint limits
        function = lambda uvw: self.distanceFromLimitSwitches(uvw)
        # Initial guess for optimization (current orientation)
        uvw0 = self.currentxyzuvw[3:5]
        # Setup bounds for each orientation parameter
        uvwbounds = [uRange, vRange, wRange]
        # Optimize to find the best uvw values using scipy minimize
        uvwoptimal = optimize.minimize(function, uvw0, args=(), method=None, jac=None, hess=None,
                      hessp=None, bounds=uvwbounds, constraints=(), tol=None,
                      callback=None, options=None)
        # Store optimized U orientation
        self.outgoingxyzuvw[3] = uvwoptimal.x[0]
        # Store optimized V orientation
        self.outgoingxyzuvw[4] = uvwoptimal.x[1]
        # Store optimized W orientation
        self.outgoingxyzuvw[5] = uvwoptimal.x[2]

    # Solve inverse kinematics with free orientation (optimized for joint limits)
    def solveInverseKinematicsFreeUVW(self, xyz, uRange, vRange, wRange):
        # Set the fixed end-effector X position
        self.outgoingxyzuvw[0] = xyz[0]
        # Set the fixed Y position
        self.outgoingxyzuvw[1] = xyz[1]
        # Set the fixed Z position
        self.outgoingxyzuvw[2] = xyz[2]
        # Iterate to find optimal orientation parameters
        self.iterate_uvw(uRange, vRange, wRange)
        # Solve inverse kinematics with optimized orientation
        return self.solveInverseKinematics(self.outgoingxyzuvw)
        
    # Check for collisions based on XYZ position with constrained orientations
    def collisionAvoidanceCheck(self, xyz):
        # Define U (roll) range in degrees: -90 to 90
        uRange = [-90, 90]
        # Define V (pitch) range: 0 to 180 degrees
        vRange = [0, 180]
        # Define W (yaw) range: 0 to 360 degrees (full rotation)
        wRange = [0, 360]
        # Collision avoidance logic would be implemented here
        # Currently a placeholder for future implementation
        pass
            
    # Solve inverse kinematics for 6-DOF robot arm
    def solveInverseKinematics(self, xyzuvw_In):
        """
        Solves the inverse kinematics problem for a 6-DOF robot arm.
        
        Calculates all possible joint angle solutions that position the end-effector
        at the desired Cartesian coordinates and orientation by sweeping wrist configuration.
        """
        
        # Initialize array to store calculated joint angles
        joints = np.zeros(self.Robot_nDOFs)
        # Initialize target pose array (will convert to radians)
        target = np.zeros(6)
        # Buffer for storing a single valid solution
        solbuffer = np.zeros(self.Robot_nDOFs)
        # Counter for number of valid solutions found
        NumberOfSol = 0
        # Index of best solution to use
        solVal = 0
        # Flag for tracking kinematic errors (0=success, 1=fail)
        KinematicError = 0
        # Output joint angles in degrees
        outgoingJointAngles = np.zeros(6)

        # Initialize the joint estimate with current configuration
        self.jointEstimate()
        
        # Store X position (mm)
        target[0] = xyzuvw_In[0]
        # Store Y position (mm)
        target[1] = xyzuvw_In[1]
        # Store Z position (mm)
        target[2] = xyzuvw_In[2]
        # Convert U rotation from degrees to radians
        target[3] = xyzuvw_In[3] * math.pi / 180
        # Convert V rotation from degrees to radians
        target[4] = xyzuvw_In[4] * math.pi / 180
        # Convert W rotation from degrees to radians
        target[5] = xyzuvw_In[5] * math.pi / 180
        
        # Search for multiple solutions by varying wrist configuration
        # Sweep joint 5 from -90 to 90 degrees in 30-degree increments
        for i in range(-3, 4):
            # Set wrist joint estimate (-90, -60, -30, 0, 30, 60, 90 degrees)
            self.joints_estimate[4] = i * 30
            
            # Attempt to solve inverse kinematics with current estimate
            success = self.inverse_kinematics_robot_xyzuvw(target, joints, self.joints_estimate)
            
            # If solution found successfully
            if success:
                # Check if this solution is different from previous solution
                if solbuffer[4] != joints[4]:
                    # Validate that all joint angles are within acceptable limits
                    if self.robot_joints_valid(joints):
                        # Store valid solution in buffer
                        for j in range(self.Robot_nDOFs):
                            solbuffer[j] = joints[j]
                            # Store in solution matrix
                            self.SolutionMatrix[j][NumberOfSol] = solbuffer[j]
                        
                        # Increment solution counter (limit to 6 solutions max)
                        if NumberOfSol <= 6:
                            NumberOfSol += 1
            else:
                # Set error flag if IK computation fails
                KinematicError = 1
        
        # Reset wrist joint estimate to original value
        self.joints_estimate[4] = self.currentJointAngles[4]
        
        # Select the best solution by comparing to current joint estimate
        # Initialize to first solution
        solVal = 0
        
        # Compare angular differences between estimate and each solution
        for i in range(self.Robot_nDOFs):
            # If first solution requires large movement and alternate exists, use second
            if (abs(self.joints_estimate[i] - self.SolutionMatrix[i][0]) > 20) and NumberOfSol > 1:
                solVal = 1
            # If second solution requires large movement, prefer first
            elif (abs(self.joints_estimate[i] - self.SolutionMatrix[i][1]) > 20) and NumberOfSol > 1:
                solVal = 0
        
        # Check if any valid solutions were found
        if NumberOfSol == 0:
            # Set error flag if no solutions exist
            KinematicError = 1
        
        # Copy selected solution to output array
        for i in range(self.Robot_nDOFs):
            outgoingJointAngles[i] = self.SolutionMatrix[i][solVal]
        # Return the best joint angle solution
        return outgoingJointAngles

    # Initialize joint estimate from current joint angles
    def jointEstimate(self):
        # Copy each current joint angle to the estimate array
        for i in range(self.Robot_nDOFs):
            self.joints_estimate[i] = self.currentJointAngles[i]
        return 
    def inverse_kinematics_robot_xyzuvw(self, target, joints, joints_estimate):
        pass
    # Solve inverse kinematics with Cartesian pose target and tool frame considerations
    def inverse_kinematics_robot(self, target, joints, joints_estimate):
        # Initialize inverse tool frame matrix
        invToolFrame = np.zeros((4, 4))
        # Initialize arm pose matrix
        pose_arm = np.zeros((4, 4))
        # Number of solutions found
        nsol = 0
        # Compute inverse of tool frame transformation matrix
        invToolFrame = np.linalg.inv(self.Robot_ToolFrame)
        # Transform target to arm's coordinate frame using base frame
        pose_arm = np.matmul(self.Robot_BaseFrame, target)
        # Apply inverse tool frame transformation
        pose_arm = np.matmul(pose_arm, invToolFrame)
        # If joint estimate provided, use it to guide solver
        if joints_estimate is not None:
            joints, nsol = self.inverse_kinematics_raw(pose_arm, joints_estimate)
        else:
            # Fallback: create approximate joint array from current joints
            joints_approx = np.zeros(6)
            # Copy current joints as approximation
            for i in range(self.Robot_nDOFs):
                joints_approx[i] = joints[i]
            # Solve with approximate initial guess
            joints, nsol = self.inverse_kinematics_raw(pose_arm, joints_approx)
        # Return failure if no solutions found
        if nsol == 0:
            return 0
        # Return success
        return 1

    # Raw inverse kinematics solver using analytical geometric method
    def inverse_kinematics_raw(self, pose, joints_approx_in):
        # Store reference to robot kinematic data
        DK = self.RobotData

        # Initialize joint angles array
        joints = [0.0] * 6
        # Initialize approximate joint angles
        joints_approx = [0.0] * 6
        # Base transformation matrix
        base = [0.0] * 16
        # Tool transformation matrix
        tool = [0.0] * 16
        # Output transformation matrix
        Hout = [0.0] * 16
        # Intermediate transformation matrix
        b_Hout = [0.0] * 9
        # Composite transformation matrix
        c_Hout = [0.0] * 16
        # Vector for wrist position calculation
        dv0 = [0.0, 0.0, -DK[33], 1.0]
        # Position vector for end-effector
        P04 = [0.0] * 4

        # Number of solutions (0 or 1)
        nsol = 0

        # Scale approximate joints using scaling factors
        for i in range(6):
            joints_approx[i] = DK[60 + i] * joints_approx_in[i]

        # Convert base frame parameters to pose matrix
        self.xyzwpr_2_pose(DK[36:42], base)
        # Convert tool frame parameters to pose matrix
        self.xyzwpr_2_pose(DK[42:48], tool)

        # Invert base transformation matrix
        for i0 in range(4):
            i = i0 << 2
            Hout[i]     = base[i0]
            Hout[i + 1] = base[i0 + 4]
            Hout[i + 2] = base[i0 + 8]
            Hout[i + 3] = base[i0 + 12]

        for i0 in range(3):
            i = i0 << 2
            Hout[i + 3] = 0.0
            # Negate rotation elements for inversion
            b_Hout[3 * i0]     = -Hout[i]
            b_Hout[3 * i0 + 1] = -Hout[i + 1]
            b_Hout[3 * i0 + 2] = -Hout[i + 2]

        for i0 in range(3):
            # Calculate inverted translation
            Hout[12 + i0] = (
                b_Hout[i0] * base[12]
                + b_Hout[i0 + 3] * base[13]
                + b_Hout[i0 + 6] * base[14]
            )

        # Invert tool transformation matrix
        for i0 in range(4):
            i = i0 << 2
            base[i]     = tool[i0]
            base[i + 1] = tool[i0 + 4]
            base[i + 2] = tool[i0 + 8]
            base[i + 3] = tool[i0 + 12]

        for i0 in range(3):
            i = i0 << 2
            base[i + 3] = 0.0
            # Negate rotation for tool inversion
            b_Hout[3 * i0]     = -base[i]
            b_Hout[3 * i0 + 1] = -base[i + 1]
            b_Hout[3 * i0 + 2] = -base[i + 2]

        for i0 in range(3):
            # Calculate inverted tool translation
            base[12 + i0] = (
                b_Hout[i0] * tool[12]
                + b_Hout[i0 + 3] * tool[13]
                + b_Hout[i0 + 6] * tool[14]
            )

        # Compute wrist position by multiplying transformations
        for i0 in range(4):
            for i in range(4):
                i1 = i << 2
                # Multiply inverted base frame with pose
                c_Hout[i0 + i1] = (
                    Hout[i0]     * pose[i1]
                    + Hout[i0+4] * pose[i1+1]
                    + Hout[i0+8] * pose[i1+2]
                    + Hout[i0+12]* pose[i1+3]
                )

            # Initialize position vector
            P04[i0] = 0.0
            for i in range(4):
                i1 = i << 2
                # Multiply with base frame
                val = (
                    c_Hout[i0]     * base[i1]
                    + c_Hout[i0+4] * base[i1+1]
                    + c_Hout[i0+8] * base[i1+2]
                    + c_Hout[i0+12]* base[i1+3]
                )
                tool[i0 + i1] = val
                # Accumulate weighted position
                P04[i0] += val * dv0[i]

        # Solve first joint angle (q1) from wrist position
        if DK[9] == 0.0:
            # Simple case: no offset in Y
            q1 = math.atan2(P04[1], P04[0])
        else:
            # Calculate discriminant for arm configuration
            disc = (P04[0]**2 + P04[1]**2) - DK[9]**2
            # If discriminant is negative, no solution exists
            if disc < 0.0:
                return joints, 0
            # Calculate q1 with shoulder offset
            q1 = math.atan2(P04[1], P04[0]) - math.atan2(DK[9], math.sqrt(disc))

        # Calculate k2 (vertical offset)
        k2 = P04[2] - DK[3]
        # Calculate k1 (horizontal reach)
        k1 = math.cos(q1) * P04[0] + math.sin(q1) * P04[1] - DK[7]

        ai = (k1*k1 + k2*k2) - DK[13]**2 - DK[21]**2 - DK[19]**2
        B = 2.0 * DK[21] * DK[13]
        C = 2.0 * DK[19] * DK[13]

        if C == 0.0:
            s31 = -ai / B
            disc = 1.0 - s31*s31
            if disc < 0.0:
                return joints, 0
            c31 = math.sqrt(disc)
        else:
            bb_cc = (B*B) / (C*C)
            disc = (2*ai*B/C/C)**2 - 4*(1+bb_cc)*((ai*ai)/(C*C)-1)
            if disc < 0.0:
                return joints, 0
            s31 = (-2*ai*B/C/C + math.sqrt(disc)) / (2*(1+bb_cc))
            c31 = (ai + B*s31) / C

        B = math.atan2(s31, c31)
        sB = math.sin(B)
        cB = math.cos(B)

        C1 = (DK[13] - DK[21]*sB) + DK[19]*cB
        C2 = DK[21]*cB + DK[19]*sB

        q1_0 = q1 - DK[2]
        q2 = math.atan2(C1*k1 - C2*k2, C1*k2 + C2*k1) - DK[8] - math.pi/2
        q3 = B - DK[14]

        # Final joint assembly
        joints[0] = q1_0
        joints[1] = q2
        joints[2] = q3
        joints[3] = joints_approx[3] - DK[20]
        joints[4] = -DK[26]
        joints[5] = -DK[32] + math.pi

        # Normalize q6
        if joints[5] > math.pi:
            joints[5] -= 2*math.pi
        elif joints[5] <= -math.pi:
            joints[5] += 2*math.pi

        # Convert to degrees & apply signs
        for i in range(6):
            joints[i] = DK[60 + i] * (joints[i] * 180.0 / math.pi)

        nsol = 1
        return joints, nsol
