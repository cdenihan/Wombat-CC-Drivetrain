//
// Created by Connor Denihan on 2/19/26.
//

#include "../include/Drivetrain.hpp"
#include <array>
#include <iostream>
#include <kipr/wombat.h>

void Drivetrain::CAMPC() // Clear all motor position counters
{
    for (int i = 0; i < 4; i++)
    {
        cmpc(this->AM[i]);
    }
}

void Drivetrain::WaitForTicksAndStop(int target_ticks, int port)
{
    while (abs(gmpc(this->AM[port])) < abs(target_ticks))
    {
        msleep(10);
    }
    ao();
}

void Drivetrain::WaitForTicksAndStop(int target_ticks,
                                     std::array<bool, 4> active_motors)
{
    int active_count = 0;
    for (bool is_active : active_motors)
    {
        if (is_active)
            active_count++;
    }

    while (active_count > 0)
    {
        for (int i = 0; i < 4; i++)
        {
            if (active_motors[i] && abs(gmpc(this->AM[i])) >= abs(target_ticks))
            {
                off(this->AM[i]);
                active_motors[i] = false;
                active_count--;
            }
        }
        msleep(10);
    }
}

Drivetrain::Drivetrain(int FL, int FR, int RL, int RR, int FL_IR_PORT,
                       int FR_IR_PORT)
    : FL(FL), FR(FR), RL(RL), RR(RR), AM{FL, FR, RL, RR}, FL_RR{FL, RR},
      FR_RL{FR, RL}, FM{FL, FR}, RM{RL, RR}, LSM{FL, RL}, RSM{FR, RR},
      FL_IR_PORT(FL_IR_PORT), FR_IR_PORT(FR_IR_PORT),
      FL_THRESHOLD(0), // Initialize to default value
      FR_THRESHOLD(0)  // Initialize to default value
{
    // Initialize performance ratings to nominal values
    this->FLP = 1.00;
    this->FRP = 1.00;
    this->RLP = 1.00;
    this->RRP = 1.00;

    this->PM[0] = this->FLP;
    this->PM[1] = this->FRP;
    this->PM[2] = this->RLP;
    this->PM[3] = this->RRP;

    // Clear all motor position counters
    CAMPC();
}

void Drivetrain::SetPerformance(double FLP, double FRP, double RLP,
                                double RRP)
{
    this->FLP = FLP;
    this->FRP = FRP;
    this->RLP = RLP;
    this->RRP = RRP;

    this->PM[0] = this->FLP;
    this->PM[1] = this->FRP;
    this->PM[2] = this->RLP;
    this->PM[3] = this->RRP;
}

void Drivetrain::SetLineTrackingThresholds(int FL_WHITE, int FR_WHITE,
                                           int FL_BLACK, int FR_BLACK)
{
    this->FL_WHITE_READING = FL_WHITE;
    this->FR_WHITE_READING = FR_WHITE;
    this->FL_BLACK_READING = FL_BLACK;
    this->FR_BLACK_READING = FR_BLACK;

    // Set thresholds as midpoint between white and black readings
    const_cast<int &>(this->FL_THRESHOLD) = (FL_WHITE + FL_BLACK) / 2;
    const_cast<int &>(this->FR_THRESHOLD) = (FR_WHITE + FR_BLACK) / 2;
}

// Positive direction is forward
void Drivetrain::Drive(int ticks, int speed)
{
    CAMPC();
    int target_ticks = abs(ticks);

    // Start motors at specified speed
    for (int i = 0; i < 4; i++)
    {
        mav(this->AM[i], (int)(speed * this->PM[i]));
    }

    // WaitForTicksAndStop(target_ticks, {true, true, true, true});
    WaitForTicksAndStop(target_ticks, 0);
    CAMPC();
}

void Drivetrain::DriveForward(int ticks, int speed) { Drive(ticks, speed); }

void Drivetrain::DriveBackward(int ticks, int speed) { Drive(ticks, -speed); }

void Drivetrain::DriveLineTracking(int ticks, int speed)
{
    CAMPC();
    int target_ticks = abs(ticks);

    while (
        abs(gmpc(this->AM[0])) <
        target_ticks) // isn't ideal as it only checks one motor, but we will deal
    {
        constexpr double TRACKED_SIDE_SPEED_PERCENTAGE = 0.75;

        int FL_READING = analog(this->FL_IR_PORT);
        int FR_READING = analog(this->FR_IR_PORT);

        bool ON_LINE_FL = FL_READING > this->FL_THRESHOLD;
        bool ON_LINE_FR = FR_READING > this->FR_THRESHOLD;

        if (ON_LINE_FL && !ON_LINE_FR)
        {
            // Left sensor on line, right sensor off line, veer right
            mav(this->AM[0], (int)(speed * this->PM[0] * TRACKED_SIDE_SPEED_PERCENTAGE));
            mav(this->AM[1], (int)(speed * this->PM[1]));
            mav(this->AM[2], (int)(speed * this->PM[2] * TRACKED_SIDE_SPEED_PERCENTAGE));
            mav(this->AM[3], (int)(speed * this->PM[3]));
        }
        else if (!ON_LINE_FL && ON_LINE_FR)
        {
            // Right sensor on line, left sensor off line, veer left
            mav(this->AM[0], (int)(speed * this->PM[0]));
            mav(this->AM[1], (int)(speed * this->PM[1] * TRACKED_SIDE_SPEED_PERCENTAGE));
            mav(this->AM[2], (int)(speed * this->PM[2]));
            mav(this->AM[3], (int)(speed * this->PM[3] * TRACKED_SIDE_SPEED_PERCENTAGE));
        }
        msleep(10);
    }
    ao();
    CAMPC();
}

void Drivetrain::DriveForwardLineTracking(int ticks, int speed)
{
    DriveLineTracking(ticks, speed);
}

// It works, but its really bad. Not reccomended to use backward line tracking
void Drivetrain::DriveBackwardLineTracking(int ticks, int speed)
{
    DriveLineTracking(ticks, -speed);
}

void Drivetrain::SquareWithLine(int speed)
{
    int FL_READING = analog(this->FL_IR_PORT);
    int FR_READING = analog(this->FR_IR_PORT);

    bool ON_LINE_FL = FL_READING > this->FL_THRESHOLD;
    bool ON_LINE_FR = FR_READING > this->FR_THRESHOLD;

    int intersects = 0;

    while (!ON_LINE_FL || !ON_LINE_FR)
    {
        if (!ON_LINE_FL && !ON_LINE_FR)
        {
            // Both sensors off line, drive straight
            for (int i = 0; i < 4; i++)
            {
                mav(this->AM[i], (int)(speed * this->PM[i]));
            }
        }
        else if (ON_LINE_FL && !ON_LINE_FR)
        {
            // Left sensor on line, right sensor off line, veer right
            for (int i = 0; i < 2; i++)
            {
                mav(this->RSM[i], (int)(speed));
            }

            for (int i = 0; i < 2; i++)
            {
                mav(this->LSM[i], (int)(-speed * 2));
            }
        }
        else if (!ON_LINE_FL && ON_LINE_FR)
        {
            // Right sensor on line, left sensor off line, veer left
            for (int i = 0; i < 2; i++)
            {
                mav(this->LSM[i], (int)(speed));
            }
            for (int i = 0; i < 2; i++)
            {
                mav(this->RSM[i], (int)(-speed * 2));
            }
        }

        else if (ON_LINE_FL && ON_LINE_FR)
        {
            // Both sensors on line drive backwards until they both aren't on the line
            // so accuratly square
            for (int i = 0; i < 4; i++)
            {
                mav(this->AM[i], (int)(-speed * this->PM[i]));
            }
        }

        FL_READING = analog(this->FL_IR_PORT);
        FR_READING = analog(this->FR_IR_PORT);

        ON_LINE_FL = FL_READING > this->FL_THRESHOLD;
        ON_LINE_FR = FR_READING > this->FR_THRESHOLD;

        if (ON_LINE_FL || ON_LINE_FR)
        {
            intersects++;
        }
    }
    ao();
}

void Drivetrain::CenterOnLine(int speed)
{
    int FL_READING = analog(this->FL_IR_PORT);
    int FR_READING = analog(this->FR_IR_PORT);

    bool ON_LINE_FL = FL_READING > this->FL_THRESHOLD;
    bool ON_LINE_FR = FR_READING > this->FR_THRESHOLD;

    while (ON_LINE_FL || ON_LINE_FR)
    {
        if (!ON_LINE_FL && !ON_LINE_FR)
        {
            break;
        }
        else if (ON_LINE_FL && !ON_LINE_FR)
        {
            // Left sensor on line, right sensor off line, veer right
            for (int i = 0; i < 2; i++)
            {
                mav(this->RSM[i], (int)(speed));
            }

            for (int i = 0; i < 2; i++)
            {
                mav(this->LSM[i], (int)(-speed * 2));
            }
        }
        else if (!ON_LINE_FL && ON_LINE_FR)
        {
            // Right sensor on line, left sensor off line, veer left
            for (int i = 0; i < 2; i++)
            {
                mav(this->LSM[i], (int)(speed));
            }
            for (int i = 0; i < 2; i++)
            {
                mav(this->RSM[i], (int)(-speed * 2));
            }
        }

        else
        {
            // Right sensor on line, left sensor off line, veer left
            for (int i = 0; i < 2; i++)
            {
                mav(this->LSM[i], (int)(speed));
            }
            for (int i = 0; i < 2; i++)
            {
                mav(this->RSM[i], (int)(-speed * 2));
            }
        }

        FL_READING = analog(this->FL_IR_PORT);
        FR_READING = analog(this->FR_IR_PORT);

        ON_LINE_FL = FL_READING > this->FL_THRESHOLD;
        ON_LINE_FR = FR_READING > this->FR_THRESHOLD;
    }
    ao();
}

void Drivetrain::DriveToLine(int speed)
{
    int FL_READING = analog(this->FL_IR_PORT);
    int FR_READING = analog(this->FR_IR_PORT);

    bool ON_LINE_FL = FL_READING > this->FL_THRESHOLD;
    bool ON_LINE_FR = FR_READING > this->FR_THRESHOLD;

    int intersects = 0;

    while (intersects < 3)
    {
        Drive(3, speed);

        FL_READING = analog(this->FL_IR_PORT);
        FR_READING = analog(this->FR_IR_PORT);

        ON_LINE_FL = FL_READING > this->FL_THRESHOLD;
        ON_LINE_FR = FR_READING > this->FR_THRESHOLD;

        if (ON_LINE_FL || ON_LINE_FR)
        {
            intersects++;
        }
    }
}

void Drivetrain::DriveForwardToLine(int speed) { DriveToLine(speed); }

void Drivetrain::DriveBackwardToLine(int speed) { DriveToLine(-speed); }

// Positive direction is right
void Drivetrain::Strafe(int ticks, int speed)
{
    CAMPC();
    int target_ticks = abs(ticks);

    // Set FL and RR positive at velocity
    mav(this->FL_RR[0], (int)(speed * this->FLP));
    mav(this->FL_RR[1], (int)(speed * this->RRP));

    // Set FR and RL negative at velocity
    mav(this->FR_RL[0], (int)(-speed * this->FRP));
    mav(this->FR_RL[1], (int)(-speed * this->RLP));

    // WaitForTicksAndStop(target_ticks, {true, true, true, true});
    WaitForTicksAndStop(target_ticks, 0);
    CAMPC();
}

void Drivetrain::StrafeLeft(int ticks, int speed) { Strafe(ticks, -speed); }

void Drivetrain::StrafeRight(int ticks, int speed) { Strafe(ticks, speed); }

void Drivetrain::StrafeToLine(int speed)
{
    int FL_READING = analog(this->FL_IR_PORT);
    int FR_READING = analog(this->FR_IR_PORT);

    bool ON_LINE_FL = FL_READING > this->FL_THRESHOLD;
    bool ON_LINE_FR = FR_READING > this->FR_THRESHOLD;

    int intersects = 0;

    while (intersects < 3)
    {
        Strafe(10, speed);

        FL_READING = analog(this->FL_IR_PORT);
        FR_READING = analog(this->FR_IR_PORT);

        ON_LINE_FL = FL_READING > this->FL_THRESHOLD;
        ON_LINE_FR = FR_READING > this->FR_THRESHOLD;

        if (ON_LINE_FL || ON_LINE_FR)
        {
            intersects++;
        }
    }
}

void Drivetrain::StrafeLeftToLine(int speed) { StrafeToLine(-speed); }

void Drivetrain::StrafeRightToLine(int speed) { StrafeToLine(speed); }

void Drivetrain::StrafeOnToLine(int speed)
{
    int FL_READING = analog(this->FL_IR_PORT);
    int FR_READING = analog(this->FR_IR_PORT);

    bool ON_LINE_FL = FL_READING > this->FL_THRESHOLD;
    bool ON_LINE_FR = FR_READING > this->FR_THRESHOLD;

    bool FL_SEEN_BLACK = false;
    bool FR_SEEN_BLACK = false;

    while (true)
    {
        Strafe(10, speed);

        FL_READING = analog(this->FL_IR_PORT);
        FR_READING = analog(this->FR_IR_PORT);

        ON_LINE_FL = FL_READING > this->FL_THRESHOLD;
        ON_LINE_FR = FR_READING > this->FR_THRESHOLD;

        if (ON_LINE_FL)
        {
            FL_SEEN_BLACK = true;
        }
        if (ON_LINE_FR)
        {
            FR_SEEN_BLACK = true;
        }
        if (FL_SEEN_BLACK && FR_SEEN_BLACK)
        {
            break;
        }
    }
}

void Drivetrain::StrafeLeftOnToLine(int speed) { StrafeOnToLine(-speed); }

void Drivetrain::StrafeRightOnToLine(int speed) { StrafeOnToLine(speed); }

// Positive ticks and postive speed are forward right
// Negative ticks and positive speed are backward right
void Drivetrain::Diagonal(int ticks, int speed)
{
    CAMPC();
    int target_ticks =
        abs(ticks); // FIX: was 'ticks', caused infinite hang on negative ticks

    std::array<bool, 4> active = {false, false, false, false};

    if (ticks > 0 && speed > 0) // Forward right (FL + RR, both forward)
    {
        mav(this->FL, (int)(speed * this->FLP));
        mav(this->RR, (int)(speed * this->RRP));
        active = {true, false, false, true}; // FL=0, RR=3
    }
    else if (ticks > 0 && speed < 0) // Forward left (FR + RL, both forward)
    {
        // FIX: was (speed * ...) which drove backward; negate so motors spin
        // forward
        mav(this->FR, (int)(-speed * this->FRP));
        mav(this->RL, (int)(-speed * this->RLP));
        active = {false, true, true, false}; // FR=1, RL=2
    }
    else if (ticks < 0 && speed > 0) // Backward right (FR + RL, both backward)
    {
        mav(this->FR, (int)(-speed * this->FRP));
        mav(this->RL, (int)(-speed * this->RLP));
        active = {false, true, true, false}; // FR=1, RL=2
    }
    else if (ticks < 0 && speed < 0) // Backward left (FL + RR, both backward)
    {
        mav(this->FL, (int)(speed * this->FLP));
        mav(this->RR, (int)(speed * this->RRP));
        active = {true, false, false, true}; // FL=0, RR=3
    }
    else
    {
        std::cout << "WARNING" << " " << "INVALID DIAGONAL INPUT" << std::endl;
        return;
    }

    WaitForTicksAndStop(target_ticks, active);
    CAMPC();
}

void Drivetrain::DriveDiagonalForwardLeft(int ticks, int speed)
{
    Diagonal(ticks, -speed);
}

void Drivetrain::DriveDiagonalForwardRight(int ticks, int speed)
{
    Diagonal(ticks, speed);
}

void Drivetrain::DriveDiagonalBackwardLeft(int ticks, int speed)
{
    Diagonal(-ticks, -speed);
}

void Drivetrain::DriveDiagonalBackwardRight(int ticks, int speed)
{
    Diagonal(-ticks, speed);
}

// Positive direction is clockwise
void Drivetrain::Rotate(int ticks, int speed)
{
    CAMPC();
    int target_ticks = abs(ticks);

    // Set LSM positive at velocity
    for (int i = 0; i < 2; i++)
    {
        mav(this->LSM[i], (int)(speed * this->PM[i]));
    }

    // Set RSM negative at velocity
    for (int i = 0; i < 2; i++)
    {
        mav(this->RSM[i], (int)(-speed * this->PM[i]));
    }

    WaitForTicksAndStop(target_ticks, {true, true, true, true});
    CAMPC();
}

void Drivetrain::RotateLeft(int ticks, int speed) { Rotate(ticks, -speed); }

void Drivetrain::RotateRight(int ticks, int speed) { Rotate(ticks, speed); }

Drivetrain::~Drivetrain() = default;