//
// Created by Connor Denihan on 2/19/26.
//

#include "../include/Wombat-CC/Drivetrain.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string_view>
#include <kipr/wombat.h>

namespace
{
    constexpr int kMotorCount = 4;
    constexpr int kControlLoopDelayMs = 10;
    constexpr int kLineIntersectionsRequired = 3;
    constexpr int kDriveToLineStepTicks = 3;
    constexpr int kStrafeToLineStepTicks = 10;
    constexpr double kTrackedSideSpeedPercentage = 0.75;

    bool IsTruthyDebugValue(const char *value)
    {
        if (value == nullptr)
        {
            return false;
        }

        const std::string_view raw_value(value);
        return raw_value == "1" || raw_value == "true" || raw_value == "TRUE" ||
               raw_value == "on" || raw_value == "ON";
    }

    bool IsDebugEnabledFromEnvironment()
    {
        return IsTruthyDebugValue(std::getenv("WOMBAT_CC_DEBUG"));
    }
} // namespace

Drivetrain::DriveByEncoderController::DriveByEncoderController(Drivetrain &parent)
    : Parent(parent)
{
}

void Drivetrain::DriveByEncoderController::Forward(int ticks, int speed)
{
    Parent.MoveDriveTicks(ticks, speed);
}

void Drivetrain::DriveByEncoderController::Backward(int ticks, int speed)
{
    Parent.MoveDriveTicks(ticks, -speed);
}

Drivetrain::DriveLineTrackingController::DriveLineTrackingController(Drivetrain &parent)
    : Parent(parent)
{
}

void Drivetrain::DriveLineTrackingController::Forward(int ticks, int speed)
{
    Parent.MoveDriveTicksLineTracking(ticks, speed);
}

void Drivetrain::DriveLineTrackingController::Backward(int ticks, int speed)
{
    Parent.MoveDriveTicksLineTracking(ticks, -speed);
}

void Drivetrain::DriveLineTrackingController::ForwardToLine(int speed)
{
    Parent.MoveDriveUntilLine(speed);
}

void Drivetrain::DriveLineTrackingController::BackwardToLine(int speed)
{
    Parent.MoveDriveUntilLine(-speed);
}

Drivetrain::StrafeByEncoderController::StrafeByEncoderController(Drivetrain &parent)
    : Parent(parent)
{
}

void Drivetrain::StrafeByEncoderController::Left(int ticks, int speed)
{
    Parent.MoveStrafeTicks(ticks, -speed);
}

void Drivetrain::StrafeByEncoderController::Right(int ticks, int speed)
{
    Parent.MoveStrafeTicks(ticks, speed);
}

Drivetrain::StrafeLineTrackingController::StrafeLineTrackingController(Drivetrain &parent)
    : Parent(parent)
{
}

void Drivetrain::StrafeLineTrackingController::Left(int ticks, int speed)
{
    Parent.MoveStrafeTicksLineTracking(ticks, -speed);
}

void Drivetrain::StrafeLineTrackingController::Right(int ticks, int speed)
{
    Parent.MoveStrafeTicksLineTracking(ticks, speed);
}

void Drivetrain::StrafeLineTrackingController::LeftToLine(int speed)
{
    Parent.MoveStrafeUntilLine(-speed);
}

void Drivetrain::StrafeLineTrackingController::RightToLine(int speed)
{
    Parent.MoveStrafeUntilLine(speed);
}

void Drivetrain::StrafeLineTrackingController::LeftOnToLine(int speed)
{
    Parent.MoveStrafeUntilBothSensorsSeeLine(-speed);
}

void Drivetrain::StrafeLineTrackingController::RightOnToLine(int speed)
{
    Parent.MoveStrafeUntilBothSensorsSeeLine(speed);
}

Drivetrain::RotateController::RotateController(Drivetrain &parent)
    : Parent(parent)
{
}

void Drivetrain::RotateController::Left(int ticks, int speed)
{
    Parent.MoveRotateTicks(ticks, speed);
}

void Drivetrain::RotateController::Right(int ticks, int speed)
{
    Parent.MoveRotateTicks(ticks, -speed);
}

Drivetrain::DiagonalController::DiagonalController(Drivetrain &parent)
    : Parent(parent)
{
}

void Drivetrain::DiagonalController::ForwardLeft(int ticks, int speed)
{
    Parent.MoveDiagonalTicks(ticks, -speed);
}

void Drivetrain::DiagonalController::ForwardRight(int ticks, int speed)
{
    Parent.MoveDiagonalTicks(ticks, speed);
}

void Drivetrain::DiagonalController::BackwardLeft(int ticks, int speed)
{
    Parent.MoveDiagonalTicks(-ticks, -speed);
}

void Drivetrain::DiagonalController::BackwardRight(int ticks, int speed)
{
    Parent.MoveDiagonalTicks(-ticks, speed);
}

Drivetrain::LineController::LineController(Drivetrain &parent)
    : Parent(parent)
{
}

void Drivetrain::LineController::Square(int speed)
{
    Parent.MoveSquareWithLine(speed);
}

void Drivetrain::LineController::Center(int speed)
{
    Parent.MoveCenterOnLine(speed);
}

Drivetrain::Drivetrain(int front_left_motor_port, int front_right_motor_port,
                       int rear_left_motor_port, int rear_right_motor_port,
                       int front_left_line_sensor_port,
                       int front_right_line_sensor_port)
    : FrontLeftMotorPort(front_left_motor_port),
      FrontRightMotorPort(front_right_motor_port),
      RearLeftMotorPort(rear_left_motor_port),
      RearRightMotorPort(rear_right_motor_port),
      AllMotorPorts{front_left_motor_port, front_right_motor_port,
                    rear_left_motor_port, rear_right_motor_port},
      FrontLeftRearRightMotors{front_left_motor_port, rear_right_motor_port},
      FrontRightRearLeftMotors{front_right_motor_port, rear_left_motor_port},
      LeftSideMotors{front_left_motor_port, rear_left_motor_port},
      RightSideMotors{front_right_motor_port, rear_right_motor_port},
      FrontLeftPerformance(1.00), FrontRightPerformance(1.00),
      RearLeftPerformance(1.00), RearRightPerformance(1.00),
      PerformanceMultipliers{1.00, 1.00, 1.00, 1.00},
      FrontLeftLineSensorPort(front_left_line_sensor_port),
      FrontRightLineSensorPort(front_right_line_sensor_port),
      FrontLeftThreshold(0), FrontRightThreshold(0), FrontLeftWhiteReading(0),
      FrontRightWhiteReading(0), FrontLeftBlackReading(0),
      FrontRightBlackReading(0),
      DebugEnabled(IsDebugEnabledFromEnvironment()),
      DriveByEncoder(*this), DriveLineTracking(*this),
      StrafeByEncoder(*this), StrafeLineTracking(*this), Rotate(*this),
      Diagonal(*this), Line(*this)
{
    RefreshPerformanceMultipliers();
    ResetMotorPositionCounters();

    if (DebugEnabled)
    {
        LogDebug("Debug mode enabled via WOMBAT_CC_DEBUG.");
    }
}

void Drivetrain::SetDebugEnabled(bool enabled)
{
    DebugEnabled = enabled;
}

bool Drivetrain::IsDebugEnabled() const
{
    return DebugEnabled;
}

void Drivetrain::SetLineTrackingThresholds(int front_left_white,
                                           int front_right_white,
                                           int front_left_black,
                                           int front_right_black)
{
    FrontLeftWhiteReading = front_left_white;
    FrontRightWhiteReading = front_right_white;
    FrontLeftBlackReading = front_left_black;
    FrontRightBlackReading = front_right_black;

    FrontLeftThreshold = (front_left_white + front_left_black) / 2;
    FrontRightThreshold = (front_right_white + front_right_black) / 2;

    if (DebugEnabled)
    {
        std::cout << "[DRIVETRAIN][DEBUG] SetLineTrackingThresholds fl="
                  << FrontLeftThreshold << " fr=" << FrontRightThreshold
                  << std::endl;
    }
}

void Drivetrain::SetPerformance(double front_left_performance,
                                double front_right_performance,
                                double rear_left_performance,
                                double rear_right_performance)
{
    FrontLeftPerformance = front_left_performance;
    FrontRightPerformance = front_right_performance;
    RearLeftPerformance = rear_left_performance;
    RearRightPerformance = rear_right_performance;

    RefreshPerformanceMultipliers();

    if (DebugEnabled)
    {
        std::cout << "[DRIVETRAIN][DEBUG] SetPerformance fl="
                  << front_left_performance << " fr="
                  << front_right_performance << " rl="
                  << rear_left_performance << " rr="
                  << rear_right_performance << std::endl;
    }
}

void Drivetrain::MoveDriveTicks(int ticks, int speed)
{
    LogCommand("MoveDriveTicks", ticks, speed);

    ResetMotorPositionCounters();
    const int target_ticks = std::abs(ticks);
    const int drive_speed = -speed;

    SetAllMotorVelocitiesScaled(drive_speed);
    WaitForTicksThenStopAll(target_ticks, 0);
    ResetMotorPositionCounters();
}

void Drivetrain::MoveDriveTicksLineTracking(int ticks, int speed)
{
    LogCommand("MoveDriveTicksLineTracking", ticks, speed);

    ResetMotorPositionCounters();
    const int target_ticks = std::abs(ticks);
    const int drive_speed = -speed;

    while (std::abs(gmpc(AllMotorPorts[0])) < target_ticks)
    {
        bool on_line_fl = false;
        bool on_line_fr = false;
        ReadLineSensorState(on_line_fl, on_line_fr);

        if (on_line_fl && !on_line_fr)
        {
            mav(AllMotorPorts[0], static_cast<int>(drive_speed * PerformanceMultipliers[0] *
                                                   kTrackedSideSpeedPercentage));
            mav(AllMotorPorts[1], static_cast<int>(drive_speed * PerformanceMultipliers[1]));
            mav(AllMotorPorts[2], static_cast<int>(drive_speed * PerformanceMultipliers[2] *
                                                   kTrackedSideSpeedPercentage));
            mav(AllMotorPorts[3], static_cast<int>(drive_speed * PerformanceMultipliers[3]));
        }
        else if (!on_line_fl && on_line_fr)
        {
            mav(AllMotorPorts[0], static_cast<int>(drive_speed * PerformanceMultipliers[0]));
            mav(AllMotorPorts[1], static_cast<int>(drive_speed * PerformanceMultipliers[1] *
                                                   kTrackedSideSpeedPercentage));
            mav(AllMotorPorts[2], static_cast<int>(drive_speed * PerformanceMultipliers[2]));
            mav(AllMotorPorts[3], static_cast<int>(drive_speed * PerformanceMultipliers[3] *
                                                   kTrackedSideSpeedPercentage));
        }

        msleep(kControlLoopDelayMs);
    }

    ao();
    ResetMotorPositionCounters();
}

void Drivetrain::MoveDriveUntilLine(int speed)
{
    LogCommand("MoveDriveUntilLine", speed);
    StepUntilLineIntersections(-speed, kDriveToLineStepTicks,
                               /*use_strafe_motion=*/false);
}

void Drivetrain::MoveStrafeTicks(int ticks, int speed)
{
    LogCommand("MoveStrafeTicks", ticks, speed);

    ResetMotorPositionCounters();
    const int target_ticks = std::abs(ticks);
    const int strafe_speed = -speed;

    mav(FrontLeftRearRightMotors[0],
        static_cast<int>(strafe_speed * FrontLeftPerformance));
    mav(FrontLeftRearRightMotors[1],
        static_cast<int>(strafe_speed * RearRightPerformance));

    mav(FrontRightRearLeftMotors[0],
        static_cast<int>(-strafe_speed * FrontRightPerformance));
    mav(FrontRightRearLeftMotors[1],
        static_cast<int>(-strafe_speed * RearLeftPerformance));

    WaitForTicksThenStopAll(target_ticks, 0);
    ResetMotorPositionCounters();
}

void Drivetrain::MoveStrafeTicksLineTracking(int ticks, int speed)
{
    LogCommand("MoveStrafeTicksLineTracking", ticks, speed);

    ResetMotorPositionCounters();
    const int target_ticks = std::abs(ticks);
    const int strafe_speed = -speed;

    while (std::abs(gmpc(AllMotorPorts[0])) < target_ticks)
    {
        bool on_line_fl = false;
        bool on_line_fr = false;
        ReadLineSensorState(on_line_fl, on_line_fr);

        double fl_scale = 1.0;
        double fr_scale = 1.0;
        double rl_scale = 1.0;
        double rr_scale = 1.0;

        if (on_line_fl && !on_line_fr)
        {
            fl_scale = kTrackedSideSpeedPercentage;
            rl_scale = kTrackedSideSpeedPercentage;
        }
        else if (!on_line_fl && on_line_fr)
        {
            fr_scale = kTrackedSideSpeedPercentage;
            rr_scale = kTrackedSideSpeedPercentage;
        }

        mav(FrontLeftMotorPort,
            static_cast<int>(strafe_speed * PerformanceMultipliers[0] * fl_scale));
        mav(FrontRightMotorPort,
            static_cast<int>(-strafe_speed * PerformanceMultipliers[1] * fr_scale));
        mav(RearLeftMotorPort,
            static_cast<int>(-strafe_speed * PerformanceMultipliers[2] * rl_scale));
        mav(RearRightMotorPort,
            static_cast<int>(strafe_speed * PerformanceMultipliers[3] * rr_scale));

        msleep(kControlLoopDelayMs);
    }

    ao();
    ResetMotorPositionCounters();
}

void Drivetrain::MoveStrafeUntilLine(int speed)
{
    LogCommand("MoveStrafeUntilLine", speed);
    StepUntilLineIntersections(speed, kStrafeToLineStepTicks,
                               /*use_strafe_motion=*/true);
}

void Drivetrain::MoveStrafeUntilBothSensorsSeeLine(int speed)
{
    LogCommand("MoveStrafeUntilBothSensorsSeeLine", speed);

    bool fl_seen_black = false;
    bool fr_seen_black = false;

    while (true)
    {
        MoveStrafeTicks(kStrafeToLineStepTicks, speed);

        bool on_line_fl = false;
        bool on_line_fr = false;
        ReadLineSensorState(on_line_fl, on_line_fr);

        if (on_line_fl)
        {
            fl_seen_black = true;
        }
        if (on_line_fr)
        {
            fr_seen_black = true;
        }
        if (fl_seen_black && fr_seen_black)
        {
            break;
        }
    }
}

void Drivetrain::MoveDiagonalTicks(int ticks, int speed)
{
    LogCommand("MoveDiagonalTicks", ticks, speed);

    ResetMotorPositionCounters();
    const int target_ticks = std::abs(ticks);

    std::array<bool, 4> active = {false, false, false, false};

    if (ticks > 0 && speed > 0)
    {
        mav(FrontLeftMotorPort,
            static_cast<int>(-speed * FrontLeftPerformance));
        mav(RearRightMotorPort,
            static_cast<int>(-speed * RearRightPerformance));
        active = {true, false, false, true};
    }
    else if (ticks > 0 && speed < 0)
    {
        mav(FrontRightMotorPort,
            static_cast<int>(speed * FrontRightPerformance));
        mav(RearLeftMotorPort, static_cast<int>(speed * RearLeftPerformance));
        active = {false, true, true, false};
    }
    else if (ticks < 0 && speed > 0)
    {
        mav(FrontRightMotorPort,
            static_cast<int>(speed * FrontRightPerformance));
        mav(RearLeftMotorPort, static_cast<int>(speed * RearLeftPerformance));
        active = {false, true, true, false};
    }
    else if (ticks < 0 && speed < 0)
    {
        mav(FrontLeftMotorPort,
            static_cast<int>(-speed * FrontLeftPerformance));
        mav(RearRightMotorPort,
            static_cast<int>(-speed * RearRightPerformance));
        active = {true, false, false, true};
    }
    else
    {
        std::cout << "[WARNING] INVALID DIAGONAL INPUT" << std::endl;
        return;
    }

    WaitForTicksThenStopActive(target_ticks, active);
    ResetMotorPositionCounters();
}

void Drivetrain::MoveRotateTicks(int ticks, int speed)
{
    LogCommand("MoveRotateTicks", ticks, speed);

    ResetMotorPositionCounters();
    const int target_ticks = std::abs(ticks);

    // Left side positive, right side negative for clockwise-positive rotation.
    mav(FrontLeftMotorPort, static_cast<int>(speed * FrontLeftPerformance));
    mav(RearLeftMotorPort, static_cast<int>(speed * RearLeftPerformance));
    mav(FrontRightMotorPort, static_cast<int>(-speed * FrontRightPerformance));
    mav(RearRightMotorPort, static_cast<int>(-speed * RearRightPerformance));

    WaitForTicksThenStopActive(target_ticks, {true, true, true, true});
    ResetMotorPositionCounters();
}

void Drivetrain::MoveSquareWithLine(int speed)
{
    LogCommand("MoveSquareWithLine", speed);

    bool on_line_fl = false;
    bool on_line_fr = false;
    ReadLineSensorState(on_line_fl, on_line_fr);

    while (!on_line_fl || !on_line_fr)
    {
        ApplyLineTrackingCorrection(speed, on_line_fl, on_line_fr,
                                    /*reverse_when_both_on_line=*/true);
        ReadLineSensorState(on_line_fl, on_line_fr);
    }

    ao();
}

void Drivetrain::MoveCenterOnLine(int speed)
{
    LogCommand("MoveCenterOnLine", speed);

    bool on_line_fl = false;
    bool on_line_fr = false;
    ReadLineSensorState(on_line_fl, on_line_fr);

    while (on_line_fl || on_line_fr)
    {
        ApplyLineTrackingCorrection(speed, on_line_fl, on_line_fr,
                                    /*reverse_when_both_on_line=*/false);
        ReadLineSensorState(on_line_fl, on_line_fr);
    }

    ao();
}

void Drivetrain::LogDebug(const char *message) const
{
    if (!DebugEnabled)
    {
        return;
    }

    std::cout << "[DRIVETRAIN][DEBUG] " << message << std::endl;
}

void Drivetrain::LogCommand(const char *command, int speed) const
{
    if (!DebugEnabled)
    {
        return;
    }

    std::cout << "[DRIVETRAIN][DEBUG] " << command << " speed=" << speed
              << std::endl;
}

void Drivetrain::LogCommand(const char *command, int ticks, int speed) const
{
    if (!DebugEnabled)
    {
        return;
    }

    std::cout << "[DRIVETRAIN][DEBUG] " << command << " ticks=" << ticks
              << " speed=" << speed << std::endl;
}

void Drivetrain::RefreshPerformanceMultipliers()
{
    PerformanceMultipliers[0] = FrontLeftPerformance;
    PerformanceMultipliers[1] = FrontRightPerformance;
    PerformanceMultipliers[2] = RearLeftPerformance;
    PerformanceMultipliers[3] = RearRightPerformance;
}

void Drivetrain::ReadLineSensorState(bool &on_line_fl, bool &on_line_fr) const
{
    const int fl_reading = analog(FrontLeftLineSensorPort);
    const int fr_reading = analog(FrontRightLineSensorPort);

    on_line_fl = fl_reading > FrontLeftThreshold;
    on_line_fr = fr_reading > FrontRightThreshold;
}

void Drivetrain::SetAllMotorVelocitiesScaled(int speed)
{
    for (int i = 0; i < kMotorCount; ++i)
    {
        mav(AllMotorPorts[i], static_cast<int>(speed * PerformanceMultipliers[i]));
    }
}

void Drivetrain::SetMotorPairVelocityRaw(const std::array<int, 2> &motors,
                                         int speed)
{
    for (int motor : motors)
    {
        mav(motor, speed);
    }
}

void Drivetrain::ApplyLineTrackingCorrection(int speed, bool on_line_fl,
                                             bool on_line_fr,
                                             bool reverse_when_both_on_line)
{
    if (!on_line_fl && !on_line_fr)
    {
        SetAllMotorVelocitiesScaled(speed);
        return;
    }

    if (on_line_fl && !on_line_fr)
    {
        SetMotorPairVelocityRaw(RightSideMotors, speed);
        SetMotorPairVelocityRaw(LeftSideMotors, -speed * 2);
        return;
    }

    if (!on_line_fl && on_line_fr)
    {
        SetMotorPairVelocityRaw(LeftSideMotors, speed);
        SetMotorPairVelocityRaw(RightSideMotors, -speed * 2);
        return;
    }

    if (reverse_when_both_on_line)
    {
        SetAllMotorVelocitiesScaled(-speed);
        return;
    }

    // Preserve CenterOnLine behavior when both sensors are on line.
    SetMotorPairVelocityRaw(LeftSideMotors, speed);
    SetMotorPairVelocityRaw(RightSideMotors, -speed * 2);
}

void Drivetrain::StepUntilLineIntersections(int speed, int step_ticks,
                                            bool use_strafe_motion)
{
    int intersections = 0;

    while (intersections < kLineIntersectionsRequired)
    {
        if (use_strafe_motion)
        {
            MoveStrafeTicks(step_ticks, speed);
        }
        else
        {
            MoveDriveTicks(step_ticks, speed);
        }

        bool on_line_fl = false;
        bool on_line_fr = false;
        ReadLineSensorState(on_line_fl, on_line_fr);

        if (on_line_fl || on_line_fr)
        {
            ++intersections;
        }
    }
}

void Drivetrain::ResetMotorPositionCounters()
{
    for (int motor : AllMotorPorts)
    {
        cmpc(motor);
    }
}

void Drivetrain::WaitForTicksThenStopAll(int target_ticks,
                                         int reference_motor_index)
{
    const int target = std::abs(target_ticks);
    while (std::abs(gmpc(AllMotorPorts[reference_motor_index])) < target)
    {
        msleep(kControlLoopDelayMs);
    }
    ao();
}

void Drivetrain::WaitForTicksThenStopActive(int target_ticks,
                                            std::array<bool, 4> active_motors)
{
    int active_count = 0;
    for (bool is_active : active_motors)
    {
        if (is_active)
        {
            ++active_count;
        }
    }

    const int target = std::abs(target_ticks);
    while (active_count > 0)
    {
        for (int i = 0; i < kMotorCount; ++i)
        {
            if (active_motors[i] && std::abs(gmpc(AllMotorPorts[i])) >= target)
            {
                off(AllMotorPorts[i]);
                active_motors[i] = false;
                --active_count;
            }
        }
        msleep(kControlLoopDelayMs);
    }
}

Drivetrain::~Drivetrain() = default;
