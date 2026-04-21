# Wombat-CC Drivetrain Library

Drivetrain is a C++ helper for a 4-motor holonomic/mecanum-style base on KIPR Wombat.

It provides:

- Encoder-based drive, strafe, rotate, and diagonal movement
- Line tracking and line acquisition helpers
- Optional debug logging

## Add To A Project

Use this dependency key in your consumer `build.zig.zon`:

```zig
.dependencies = .{
    .wombat_cc_lib_drivetrain = .{ .path = "../Wombat-CC-Drivetrain/" },
};
```

Then include the header:

```cpp
#include <Wombat-CC/Drivetrain.hpp>
```

## Motor Layout And Constructor

Constructor:

```cpp
Drivetrain(
    int FL, int FR, int RL, int RR
)
```

Then configure line-tracking IR ports separately:

```cpp
drivetrain.ConfigureLineTrackingSensors(int FL_IR_PORT, int FR_IR_PORT);
```

Port meaning:

- `FL`: front-left motor
- `FR`: front-right motor
- `RL`: rear-left motor
- `RR`: rear-right motor

Line sensor meaning:

- `FL_IR_PORT`: front-left line sensor analog port
- `FR_IR_PORT`: front-right line sensor analog port

## Quick Start

```cpp
#include <Wombat-CC/Drivetrain.hpp>

int main()
{
    Drivetrain drivetrain(0, 1, 2, 3);

    drivetrain.SetDebugEnabled(true);

    drivetrain.SetPerformance(1.0, 1.0, 1.0, 1.0);
    drivetrain.ConfigureLineTrackingSensors(0, 1);
    drivetrain.SetLineTrackingThresholds(200, 200, 3600, 3600);

    drivetrain.DriveByEncoder.Forward(300, 800);
    drivetrain.StrafeByEncoder.Right(300, 800);
    drivetrain.Rotate.Right(200, 700);
    drivetrain.DriveLineTracking.Forward(1000, 700);
    drivetrain.Line.Square(500);

    return 0;
}
```

## Recommended Initialization Order

1. Construct drivetrain with motor ports.
2. Call `SetPerformance(...)` to tune per-motor multipliers.
3. Call `ConfigureLineTrackingSensors(...)`.
4. Call `SetLineTrackingThresholds(...)` before any line-based movement.

## API Overview

Movement is grouped by domain for discoverability and future expansion.

### Configuration

- `void SetPerformance(double FLP, double FRP, double RLP, double RRP)`
- `void ConfigureLineTrackingSensors(int FL_IR_PORT, int FR_IR_PORT)`
- `void SetLineTrackingThresholds(int FL_white, int FR_white, int FL_black, int FR_black)`
- `void SetDebugEnabled(bool enabled)`
- `bool IsDebugEnabled() const`
- `bool IsLineTrackingConfigured() const`

### DriveByEncoder Group

- `drivetrain.DriveByEncoder.Forward(int ticks, int speed)`
- `drivetrain.DriveByEncoder.Backward(int ticks, int speed)`

### DriveLineTracking Group

- `drivetrain.DriveLineTracking.Forward(int ticks, int speed)`
- `drivetrain.DriveLineTracking.Backward(int ticks, int speed)`
- `drivetrain.DriveLineTracking.ForwardToLine(int speed)`
- `drivetrain.DriveLineTracking.BackwardToLine(int speed)`

### StrafeByEncoder Group

- `drivetrain.StrafeByEncoder.Left(int ticks, int speed)`
- `drivetrain.StrafeByEncoder.Right(int ticks, int speed)`

### StrafeLineTracking Group

- `drivetrain.StrafeLineTracking.Left(int ticks, int speed)`
- `drivetrain.StrafeLineTracking.Right(int ticks, int speed)`
- `drivetrain.StrafeLineTracking.LeftToLine(int speed)`
- `drivetrain.StrafeLineTracking.RightToLine(int speed)`
- `drivetrain.StrafeLineTracking.LeftOnToLine(int speed)`
- `drivetrain.StrafeLineTracking.RightOnToLine(int speed)`

### Rotate Group

- `drivetrain.Rotate.Left(int ticks, int speed)`
- `drivetrain.Rotate.Right(int ticks, int speed)`

### Diagonal Group

- `drivetrain.Diagonal.ForwardLeft(int ticks, int speed)`
- `drivetrain.Diagonal.ForwardRight(int ticks, int speed)`
- `drivetrain.Diagonal.BackwardLeft(int ticks, int speed)`
- `drivetrain.Diagonal.BackwardRight(int ticks, int speed)`

### Line Group

- `drivetrain.Line.Square(int speed)`
- `drivetrain.Line.Center(int speed)`

### Legacy Note

The old flat movement API has been removed in favor of the grouped API above.

## Direction Conventions

- Drive: positive speed is forward.
- Strafe: positive speed is right.
- Rotate: positive speed is clockwise.
- Diagonal wrappers handle sign combinations internally.

## Debug Logging

Two options are supported:

- Runtime toggle in code with `SetDebugEnabled(true)`
- Environment variable before program start: `WOMBAT_CC_DEBUG=1`

Truthy values currently recognized:

- `1`
- `true`
- `TRUE`
- `on`
- `ON`

## Practical Notes

- Line threshold values are computed internally as midpoint between white and black readings.
- Line-based movement methods require `ConfigureLineTrackingSensors(...)` and `SetLineTrackingThresholds(...)`; otherwise they print a warning and return.
- Performance multipliers help compensate for uneven motors or drivetrain friction.
- Some encoder-based primitives use one reference encoder for completion checks, so per-motor performance tuning is important for straightness.
- Backward line tracking is available, but forward line tracking generally gives more stable results.
