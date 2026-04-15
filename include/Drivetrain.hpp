//
// Created by Connor Denihan on 2/19/26.
//

#ifndef WOMBAT_CC_Drivetrain_HPP
#define WOMBAT_CC_Drivetrain_HPP

#include <array>

class Drivetrain
{
public:
    class DriveByEncoderController
    {
    public:
        explicit DriveByEncoderController(Drivetrain &parent);

        void Forward(int ticks, int speed);
        void Backward(int ticks, int speed);

    private:
        Drivetrain &Parent;
    };

    class DriveLineTrackingController
    {
    public:
        explicit DriveLineTrackingController(Drivetrain &parent);

        void Forward(int ticks, int speed);
        void Backward(int ticks, int speed);
        void ForwardToLine(int speed);
        void BackwardToLine(int speed);

    private:
        Drivetrain &Parent;
    };

    class StrafeByEncoderController
    {
    public:
        explicit StrafeByEncoderController(Drivetrain &parent);

        void Left(int ticks, int speed);
        void Right(int ticks, int speed);

    private:
        Drivetrain &Parent;
    };

    class StrafeLineTrackingController
    {
    public:
        explicit StrafeLineTrackingController(Drivetrain &parent);

        void Left(int ticks, int speed);
        void Right(int ticks, int speed);
        void LeftToLine(int speed);
        void RightToLine(int speed);
        void LeftOnToLine(int speed);
        void RightOnToLine(int speed);

    private:
        Drivetrain &Parent;
    };

    class RotateController
    {
    public:
        explicit RotateController(Drivetrain &parent);

        void Left(int ticks, int speed);
        void Right(int ticks, int speed);

    private:
        Drivetrain &Parent;
    };

    class DiagonalController
    {
    public:
        explicit DiagonalController(Drivetrain &parent);

        void ForwardLeft(int ticks, int speed);
        void ForwardRight(int ticks, int speed);
        void BackwardLeft(int ticks, int speed);
        void BackwardRight(int ticks, int speed);

    private:
        Drivetrain &Parent;
    };

    class LineController
    {
    public:
        explicit LineController(Drivetrain &parent);

        void Square(int speed);
        void Center(int speed);

    private:
        Drivetrain &Parent;
    };

    Drivetrain(int front_left_motor_port, int front_right_motor_port,
               int rear_left_motor_port, int rear_right_motor_port,
               int front_left_line_sensor_port,
               int front_right_line_sensor_port);

    void SetDebugEnabled(bool enabled);
    bool IsDebugEnabled() const;
    void SetLineTrackingThresholds(int front_left_white,
                                   int front_right_white,
                                   int front_left_black,
                                   int front_right_black);
    void SetPerformance(double front_left_performance,
                        double front_right_performance,
                        double rear_left_performance,
                        double rear_right_performance);

    ~Drivetrain();

private:
    // Motor ports (FL, FR, RL, RR)
    const int FrontLeftMotorPort;
    const int FrontRightMotorPort;
    const int RearLeftMotorPort;
    const int RearRightMotorPort;

    // Useful motor groupings
    const std::array<int, 4> AllMotorPorts;
    const std::array<int, 2> FrontLeftRearRightMotors;
    const std::array<int, 2> FrontRightRearLeftMotors;
    const std::array<int, 2> LeftSideMotors;
    const std::array<int, 2> RightSideMotors;

    // Per-motor performance multipliers
    double FrontLeftPerformance;
    double FrontRightPerformance;
    double RearLeftPerformance;
    double RearRightPerformance;
    std::array<double, 4> PerformanceMultipliers;

    // Line sensor ports and thresholds
    const int FrontLeftLineSensorPort;
    const int FrontRightLineSensorPort;
    int FrontLeftThreshold;
    int FrontRightThreshold;
    int FrontLeftWhiteReading;
    int FrontRightWhiteReading;
    int FrontLeftBlackReading;
    int FrontRightBlackReading;

    bool DebugEnabled;

    // Internal motion helpers
    void MoveDriveTicks(int ticks, int speed);
    void MoveDriveTicksLineTracking(int ticks, int speed);
    void MoveDriveUntilLine(int speed);
    void MoveStrafeTicks(int ticks, int speed);
    void MoveStrafeTicksLineTracking(int ticks, int speed);
    void MoveStrafeUntilLine(int speed);
    void MoveStrafeUntilBothSensorsSeeLine(int speed);
    void MoveDiagonalTicks(int ticks, int speed);
    void MoveRotateTicks(int ticks, int speed);
    void MoveSquareWithLine(int speed);
    void MoveCenterOnLine(int speed);

    // Internal utility helpers
    void LogDebug(const char *message) const;
    void LogCommand(const char *command, int speed) const;
    void LogCommand(const char *command, int ticks, int speed) const;
    void RefreshPerformanceMultipliers();
    void ReadLineSensorState(bool &on_line_fl, bool &on_line_fr) const;
    void SetAllMotorVelocitiesScaled(int speed);
    void SetMotorPairVelocityRaw(const std::array<int, 2> &motors, int speed);
    void ApplyLineTrackingCorrection(int speed, bool on_line_fl,
                                     bool on_line_fr,
                                     bool reverse_when_both_on_line);
    void StepUntilLineIntersections(int speed, int step_ticks,
                                    bool use_strafe_motion);
    void ResetMotorPositionCounters();
    void WaitForTicksThenStopAll(int target_ticks, int reference_motor_index);
    void WaitForTicksThenStopActive(int target_ticks,
                                    std::array<bool, 4> active_motors);

public:
    // Grouped API entry points.
    DriveByEncoderController DriveByEncoder;
    DriveLineTrackingController DriveLineTracking;
    StrafeByEncoderController StrafeByEncoder;
    StrafeLineTrackingController StrafeLineTracking;
    RotateController Rotate;
    DiagonalController Diagonal;
    LineController Line;
};

#endif // WOMBAT_CC_Drivetrain_HPP
