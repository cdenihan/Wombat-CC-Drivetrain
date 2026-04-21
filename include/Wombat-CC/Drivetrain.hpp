//
// Created by Connor Denihan on 2/19/26.
//

/**
 * @file Drivetrain.hpp
 * @brief Holonomic/mecanum drivetrain helper for KIPR Wombat projects.
 *
 * This header exposes grouped movement controllers for encoder-based motion,
 * line tracking, strafing, rotation, and diagonal movement.
 */

#ifndef WOMBAT_CC_Drivetrain_HPP
#define WOMBAT_CC_Drivetrain_HPP

#include <array>

/**
 * @brief High-level controller for a 4-motor holonomic/mecanum drivetrain.
 *
 * The API is grouped into nested controller objects (for example
 * DriveByEncoder, StrafeByEncoder, Rotate) to keep call sites explicit and
 * discoverable.
 */
class Drivetrain
{
public:
    /**
     * @brief Drive (forward/backward) controller using encoder ticks.
     */
    class DriveByEncoderController
    {
    public:
        /**
         * @brief Construct a drive-by-encoder controller.
         * @param parent Owning drivetrain.
         */
        explicit DriveByEncoderController(Drivetrain &parent);

        /**
         * @brief Move the drivetrain forward by encoder ticks.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void Forward(int ticks, int speed);

        /**
         * @brief Move the drivetrain backward by encoder ticks.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void Backward(int ticks, int speed);

    private:
        /** @brief Owning drivetrain reference. */
        Drivetrain &Parent;
    };

    /**
     * @brief Drive controller with line-tracking correction.
     */
    class DriveLineTrackingController
    {
    public:
        /**
         * @brief Construct a drive line-tracking controller.
         * @param parent Owning drivetrain.
         */
        explicit DriveLineTrackingController(Drivetrain &parent);

        /**
         * @brief Move forward with line-tracking correction.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void Forward(int ticks, int speed);

        /**
         * @brief Move backward with line-tracking correction.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void Backward(int ticks, int speed);

        /**
         * @brief Drive forward until line intersections are detected.
         * @param speed Requested speed magnitude.
         */
        void ForwardToLine(int speed);

        /**
         * @brief Drive backward until line intersections are detected.
         * @param speed Requested speed magnitude.
         */
        void BackwardToLine(int speed);

    private:
        /** @brief Owning drivetrain reference. */
        Drivetrain &Parent;
    };

    /**
     * @brief Strafe (left/right) controller using encoder ticks.
     */
    class StrafeByEncoderController
    {
    public:
        /**
         * @brief Construct a strafe-by-encoder controller.
         * @param parent Owning drivetrain.
         */
        explicit StrafeByEncoderController(Drivetrain &parent);

        /**
         * @brief Strafe left by encoder ticks.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void Left(int ticks, int speed);

        /**
         * @brief Strafe right by encoder ticks.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void Right(int ticks, int speed);

    private:
        /** @brief Owning drivetrain reference. */
        Drivetrain &Parent;
    };

    /**
     * @brief Strafe controller with line-tracking helpers.
     */
    class StrafeLineTrackingController
    {
    public:
        /**
         * @brief Construct a strafe line-tracking controller.
         * @param parent Owning drivetrain.
         */
        explicit StrafeLineTrackingController(Drivetrain &parent);

        /**
         * @brief Strafe left with line-tracking correction.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void Left(int ticks, int speed);

        /**
         * @brief Strafe right with line-tracking correction.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void Right(int ticks, int speed);

        /**
         * @brief Strafe left until the configured line condition is reached.
         * @param speed Requested speed magnitude.
         */
        void LeftToLine(int speed);

        /**
         * @brief Strafe right until the configured line condition is reached.
         * @param speed Requested speed magnitude.
         */
        void RightToLine(int speed);

        /**
         * @brief Strafe left until both sensors have seen line.
         * @param speed Requested speed magnitude.
         */
        void LeftOnToLine(int speed);

        /**
         * @brief Strafe right until both sensors have seen line.
         * @param speed Requested speed magnitude.
         */
        void RightOnToLine(int speed);

    private:
        /** @brief Owning drivetrain reference. */
        Drivetrain &Parent;
    };

    /**
     * @brief Rotation controller using encoder ticks.
     */
    class RotateController
    {
    public:
        /**
         * @brief Construct a rotation controller.
         * @param parent Owning drivetrain.
         */
        explicit RotateController(Drivetrain &parent);

        /**
         * @brief Rotate left by encoder ticks.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void Left(int ticks, int speed);

        /**
         * @brief Rotate right by encoder ticks.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void Right(int ticks, int speed);

    private:
        /** @brief Owning drivetrain reference. */
        Drivetrain &Parent;
    };

    /**
     * @brief Diagonal movement controller using encoder ticks.
     */
    class DiagonalController
    {
    public:
        /**
         * @brief Construct a diagonal movement controller.
         * @param parent Owning drivetrain.
         */
        explicit DiagonalController(Drivetrain &parent);

        /**
         * @brief Move diagonally forward-left.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void ForwardLeft(int ticks, int speed);

        /**
         * @brief Move diagonally forward-right.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void ForwardRight(int ticks, int speed);

        /**
         * @brief Move diagonally backward-left.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void BackwardLeft(int ticks, int speed);

        /**
         * @brief Move diagonally backward-right.
         * @param ticks Target encoder ticks.
         * @param speed Requested speed magnitude.
         */
        void BackwardRight(int ticks, int speed);

    private:
        /** @brief Owning drivetrain reference. */
        Drivetrain &Parent;
    };

    /**
     * @brief Line alignment controller.
     */
    class LineController
    {
    public:
        /**
         * @brief Construct a line alignment controller.
         * @param parent Owning drivetrain.
         */
        explicit LineController(Drivetrain &parent);

        /**
         * @brief Square the drivetrain against a line.
         * @param speed Requested speed magnitude.
         */
        void Square(int speed);

        /**
         * @brief Center the drivetrain relative to a line.
         * @param speed Requested speed magnitude.
         */
        void Center(int speed);

    private:
        /** @brief Owning drivetrain reference. */
        Drivetrain &Parent;
    };

    /**
     * @brief Construct a drivetrain controller.
     * @param front_left_motor_port Front-left motor port.
     * @param front_right_motor_port Front-right motor port.
     * @param rear_left_motor_port Rear-left motor port.
     * @param rear_right_motor_port Rear-right motor port.
     */
    Drivetrain(int front_left_motor_port, int front_right_motor_port,
               int rear_left_motor_port, int rear_right_motor_port);

    /**
     * @brief Configure analog ports used for line tracking.
     * @param front_left_line_sensor_port Front-left line sensor analog port.
     * @param front_right_line_sensor_port Front-right line sensor analog port.
     */
    void ConfigureLineTrackingSensors(int front_left_line_sensor_port,
                                      int front_right_line_sensor_port);

    /**
     * @brief Query whether line tracking is fully configured.
     * @return True when sensor ports and thresholds are configured.
     */
    bool IsLineTrackingConfigured() const;

    /**
     * @brief Enable or disable debug logging.
     * @param enabled True to enable debug logs.
     */
    void SetDebugEnabled(bool enabled);

    /**
     * @brief Query current debug logging state.
     * @return True when debug logging is enabled.
     */
    bool IsDebugEnabled() const;

    /**
     * @brief Configure white/black calibration values for line sensors.
     * @param front_left_white Front-left white-surface reading.
     * @param front_right_white Front-right white-surface reading.
     * @param front_left_black Front-left black-line reading.
     * @param front_right_black Front-right black-line reading.
     */
    void SetLineTrackingThresholds(int front_left_white,
                                   int front_right_white,
                                   int front_left_black,
                                   int front_right_black);

    /**
     * @brief Configure per-motor performance multipliers.
     * @param front_left_performance Front-left multiplier.
     * @param front_right_performance Front-right multiplier.
     * @param rear_left_performance Rear-left multiplier.
     * @param rear_right_performance Rear-right multiplier.
     */
    void SetPerformance(double front_left_performance,
                        double front_right_performance,
                        double rear_left_performance,
                        double rear_right_performance);

    /** @brief Destroy the drivetrain controller. */
    ~Drivetrain();

private:
    /** @brief Front-left motor port. */
    const int FrontLeftMotorPort;
    /** @brief Front-right motor port. */
    const int FrontRightMotorPort;
    /** @brief Rear-left motor port. */
    const int RearLeftMotorPort;
    /** @brief Rear-right motor port. */
    const int RearRightMotorPort;

    /** @brief All motor ports in FL, FR, RL, RR order. */
    const std::array<int, 4> AllMotorPorts;
    /** @brief FL and RR motor ports. */
    const std::array<int, 2> FrontLeftRearRightMotors;
    /** @brief FR and RL motor ports. */
    const std::array<int, 2> FrontRightRearLeftMotors;
    /** @brief Left-side motor ports. */
    const std::array<int, 2> LeftSideMotors;
    /** @brief Right-side motor ports. */
    const std::array<int, 2> RightSideMotors;

    /** @brief Front-left motor performance multiplier. */
    double FrontLeftPerformance;
    /** @brief Front-right motor performance multiplier. */
    double FrontRightPerformance;
    /** @brief Rear-left motor performance multiplier. */
    double RearLeftPerformance;
    /** @brief Rear-right motor performance multiplier. */
    double RearRightPerformance;
    /** @brief Cached FL, FR, RL, RR multipliers. */
    std::array<double, 4> PerformanceMultipliers;

    /** @brief Front-left line sensor analog port. */
    int FrontLeftLineSensorPort;
    /** @brief Front-right line sensor analog port. */
    int FrontRightLineSensorPort;
    /** @brief Front-left line threshold. */
    int FrontLeftThreshold;
    /** @brief Front-right line threshold. */
    int FrontRightThreshold;
    /** @brief Front-left calibrated white reading. */
    int FrontLeftWhiteReading;
    /** @brief Front-right calibrated white reading. */
    int FrontRightWhiteReading;
    /** @brief Front-left calibrated black reading. */
    int FrontLeftBlackReading;
    /** @brief Front-right calibrated black reading. */
    int FrontRightBlackReading;
    /** @brief True once sensor ports are configured. */
    bool LineSensorsConfigured;
    /** @brief True once line thresholds are configured. */
    bool LineTrackingThresholdsConfigured;

    /** @brief Runtime debug logging flag. */
    bool DebugEnabled;

    /**
     * @brief Internal drive primitive using encoder ticks.
     * @param ticks Target encoder ticks.
     * @param speed Signed speed command.
     */
    void MoveDriveTicks(int ticks, int speed);

    /**
     * @brief Internal drive primitive with line-tracking correction.
     * @param ticks Target encoder ticks.
     * @param speed Signed speed command.
     */
    void MoveDriveTicksLineTracking(int ticks, int speed);

    /**
     * @brief Internal drive-until-line primitive.
     * @param speed Signed speed command.
     */
    void MoveDriveUntilLine(int speed);

    /**
     * @brief Internal strafe primitive using encoder ticks.
     * @param ticks Target encoder ticks.
     * @param speed Signed speed command.
     */
    void MoveStrafeTicks(int ticks, int speed);

    /**
     * @brief Internal strafe primitive with line-tracking correction.
     * @param ticks Target encoder ticks.
     * @param speed Signed speed command.
     */
    void MoveStrafeTicksLineTracking(int ticks, int speed);

    /**
     * @brief Internal strafe-until-line primitive.
     * @param speed Signed speed command.
     */
    void MoveStrafeUntilLine(int speed);

    /**
     * @brief Internal strafe primitive until both sensors see line.
     * @param speed Signed speed command.
     */
    void MoveStrafeUntilBothSensorsSeeLine(int speed);

    /**
     * @brief Internal diagonal movement primitive.
     * @param ticks Target encoder ticks.
     * @param speed Signed speed command.
     */
    void MoveDiagonalTicks(int ticks, int speed);

    /**
     * @brief Internal rotation primitive.
     * @param ticks Target encoder ticks.
     * @param speed Signed speed command.
     */
    void MoveRotateTicks(int ticks, int speed);

    /**
     * @brief Internal line-squaring primitive.
     * @param speed Signed speed command.
     */
    void MoveSquareWithLine(int speed);

    /**
     * @brief Internal line-centering primitive.
     * @param speed Signed speed command.
     */
    void MoveCenterOnLine(int speed);

    /**
     * @brief Emit a debug log line when debug mode is enabled.
     * @param message Message to print.
     */
    void LogDebug(const char *message) const;

    /**
     * @brief Emit a debug command log with speed.
     * @param command Command name.
     * @param speed Speed argument.
     */
    void LogCommand(const char *command, int speed) const;

    /**
     * @brief Emit a debug command log with ticks and speed.
     * @param command Command name.
     * @param ticks Tick argument.
     * @param speed Speed argument.
     */
    void LogCommand(const char *command, int ticks, int speed) const;

    /**
     * @brief Check whether line tracking methods are available.
     * @param operation Name of the attempted line-tracking operation.
     * @return True when line tracking is fully configured.
     */
    bool EnsureLineTrackingConfigured(const char *operation) const;

    /** @brief Refresh cached performance multiplier array values. */
    void RefreshPerformanceMultipliers();

    /**
     * @brief Read line sensor states against configured thresholds.
     * @param on_line_fl Output front-left on-line state.
     * @param on_line_fr Output front-right on-line state.
     */
    void ReadLineSensorState(bool &on_line_fl, bool &on_line_fr) const;

    /**
     * @brief Apply the same signed speed to all motors using multipliers.
     * @param speed Signed speed command.
     */
    void SetAllMotorVelocitiesScaled(int speed);

    /**
     * @brief Set a raw speed for a two-motor pair.
     * @param motors Pair of motor ports.
     * @param speed Signed raw speed value.
     */
    void SetMotorPairVelocityRaw(const std::array<int, 2> &motors, int speed);

    /**
     * @brief Apply line-tracking steering correction.
     * @param speed Base signed speed command.
     * @param on_line_fl Front-left sensor on-line state.
     * @param on_line_fr Front-right sensor on-line state.
     * @param reverse_when_both_on_line Reverse behavior when both are on line.
     */
    void ApplyLineTrackingCorrection(int speed, bool on_line_fl,
                                     bool on_line_fr,
                                     bool reverse_when_both_on_line);

    /**
     * @brief Step motion until a line intersection count target is reached.
     * @param speed Signed speed command.
     * @param step_ticks Tick distance for each step.
     * @param use_strafe_motion True to step using strafe, false for drive.
     */
    void StepUntilLineIntersections(int speed, int step_ticks,
                                    bool use_strafe_motion);

    /** @brief Reset all motor encoder position counters to zero. */
    void ResetMotorPositionCounters();

    /**
     * @brief Wait for a reference encoder target, then stop all motors.
     * @param target_ticks Absolute target ticks.
     * @param reference_motor_index Index into AllMotorPorts.
     */
    void WaitForTicksThenStopAll(int target_ticks, int reference_motor_index);

    /**
     * @brief Stop each active motor after it reaches target encoder ticks.
     * @param target_ticks Absolute target ticks.
     * @param active_motors Active flags in FL, FR, RL, RR order.
     */
    void WaitForTicksThenStopActive(int target_ticks,
                                    std::array<bool, 4> active_motors);

public:
    /** @brief Grouped drive movement API. */
    DriveByEncoderController DriveByEncoder;
    /** @brief Grouped drive + line tracking API. */
    DriveLineTrackingController DriveLineTracking;
    /** @brief Grouped strafe movement API. */
    StrafeByEncoderController StrafeByEncoder;
    /** @brief Grouped strafe + line tracking API. */
    StrafeLineTrackingController StrafeLineTracking;
    /** @brief Grouped rotation API. */
    RotateController Rotate;
    /** @brief Grouped diagonal movement API. */
    DiagonalController Diagonal;
    /** @brief Grouped line alignment API. */
    LineController Line;
};

#endif // WOMBAT_CC_Drivetrain_HPP
