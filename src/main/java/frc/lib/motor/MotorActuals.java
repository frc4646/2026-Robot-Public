package frc.lib.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

// @AutoLog
public class MotorActuals
{
    public double dutyCycle = 0.0;
    public double velocityRotationsPerSecond = 0.0;
    public double positionRotations = 0.0;

    public double velocityInchesPerSecond = 0.0;
    public double positionInches = 0.0;

    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
    public double torqueCurrentAmps = 0.0;

    public boolean softLimitForward = false;
    public boolean softLimitReverse = false;

    public double getDutyCycle()
    {
        return dutyCycle;
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

    public Voltage getAppliedVoltage()
    {
        return Volts.of(appliedVolts);
    }

    public Current getStatorCurrent()
    {
        return Amps.of(currentStatorAmps);
    }

    public Current getSupplyCurrent()
    {
        return Amps.of(currentSupplyAmps);
    }

    public Current getTorqueCurrent()
    {
        return Amps.of(torqueCurrentAmps);
    }
}
