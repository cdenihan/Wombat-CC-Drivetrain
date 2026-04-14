//
// Created by Connor Denihan on 2/19/26.
//

#ifndef WOMBAT_CC_Drivetrain_HPP
#define WOMBAT_CC_Drivetrain_HPP

#include <array>

class Drivetrain
{
private:
    // Motor port definitions
    // Defined like how grid quadrants are
    // FL (Front Left), FR (Front Right), RL (Rear Left), RR (Rear Right)
    const int FL; // Front Left
    const int FR; // Front Right
    const int RL; // Rear Left
    const int RR; // Rear Right

    // Arrays to store motors
    const int AM[4];    // All Motors
    const int FL_RR[2]; // Front Left and Rear Right
    const int FR_RL[2]; // Front Right and Rear Left
    const int FM[2];    // Front Motors
    const int RM[2];    // Rear Motors
    const int LSM[2];   // Left Side Motors
    const int RSM[2];   // Right Side Motors

    // Per motor performance ratings (1.0f = nominal)
    double FLP;   // Front Left Performance Rating
    double FRP;   // Front Right Performance Rating
    double RLP;   // Rear Left Performance Rating
    double RRP;   // Rear Right Performance Rating
    double PM[4]; // Performance Ratings Array

    // Wheel information
    double TPR; // Ticks per Revolution

    // Line tracking thresholds
    const int FL_IR_PORT; // Front Left IR Sensor Port
    const int FR_IR_PORT; // Front Right IR Sensor Port

    const int FL_THRESHOLD; // Front Left IR Sensor Threshold
    const int FR_THRESHOLD; // Front Right IR Sensor Threshold
    int FL_WHITE_READING;   // Front Left IR Sensor White Reading
    int FR_WHITE_READING;   // Front Right IR Sensor White Reading
    int FL_BLACK_READING;   // Front Left IR Sensor Black Reading
    int FR_BLACK_READING;   // Front Right IR Sensor Black Reading

    // "Action" functions that define how to do something
    // The goal of these functions is to reduce code duplication
    void CAMPC(); // Clear all motor position counters
    void WaitForTicksAndStop(int target_ticks, int port);
    void WaitForTicksAndStop(int target_ticks, std::array<bool, 4> active_motors);

    void Drive(int ticks, int speed);
    void DriveLineTracking(int ticks, int speed);
    void DriveToLine(int speed);
    void StrafeToLine(int speed);
    void StrafeOnToLine(int speed);
    void Strafe(int ticks, int speed);
    void Diagonal(int ticks, int speed);
    void Rotate(int ticks, int speed);

public:
    // Initalizer functions
    Drivetrain(int FL, int FR, int RL, int RR, int FL_IR_PORT, int FR_IR_PORT);
    void SetLineTrackingThresholds(int FL_white, int FR_white, int FL_black,
                                   int FR_black);
    void SetPerformance(double FLP, double FRP, double RLP, double RRP);

    // Drive Functions (Encoder)
    void DriveForward(int ticks, int speed);
    void DriveBackward(int ticks, int speed);
    void StrafeLeft(int ticks, int speed);
    void StrafeRight(int ticks, int speed);
    void DriveDiagonalForwardLeft(int ticks, int speed);
    void DriveDiagonalForwardRight(int ticks, int speed);
    void DriveDiagonalBackwardLeft(int ticks, int speed);
    void DriveDiagonalBackwardRight(int ticks, int speed);

    // Drive Functions (Line Tracking)
    void DriveForwardLineTracking(int ticks, int speed);
    void DriveBackwardLineTracking(int ticks, int speed);
    void StrafeLeftToLine(int speed);
    void StrafeRightToLine(int speed);
    void StrafeLeftOnToLine(int speed);
    void StrafeRightOnToLine(int speed);
    void DriveForwardToLine(int speed);
    void DriveBackwardToLine(int speed);

    // Rotation Functions
    void RotateLeft(int ticks, int speed);
    void RotateRight(int ticks, int speed);

    void SquareWithLine(int speed);
    void CenterOnLine(int speed);

    // Destructor
    ~Drivetrain();
};

#endif // WOMBAT_CC_Drivetrain_HPP
