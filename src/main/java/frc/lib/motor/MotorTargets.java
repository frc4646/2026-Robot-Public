package frc.lib.motor;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

// @AutoLog
public class MotorTargets
{
    public static enum TargetModes
    {
        Voltage,
        Velocity,
        Position,
        DutyCycle
    }

    public TargetModes mode = TargetModes.Voltage;

    public double velocityRotationsPerSecond = 0.0;
    public double velocityInchesPerSecond = 0.0;

    public double positionRotations = 0.0;
    public double positionInches = 0.0;

    public double voltage = 0.0;
    public double dutyCycle = 0.0;

    public TargetModes getMode()
    {
        return mode;
    }

    public AngularVelocity getAngularVelocity()
    {
        return RotationsPerSecond.of(velocityRotationsPerSecond);
    }

    public Angle getAngle()
    {
        return Rotations.of(positionRotations);
    }

    public LinearVelocity getLinearVelocity()
    {
        return InchesPerSecond.of(velocityInchesPerSecond);
    }

    public Distance getDistance()
    {
        return Inches.of(positionInches);
    }

}
