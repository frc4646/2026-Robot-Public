package frc.lib.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;

public class DistanceAngleConverter
{
    private Distance rollerRadius;

    public static DistanceAngleConverter getDefault()
    {
        return new DistanceAngleConverter(Inches.of(1.0));
    }

    // Constructor
    public DistanceAngleConverter(Distance radius)
    {
        this.rollerRadius = radius;
    }

    public Distance getRadius()
    {
        return rollerRadius;
    }

    /**
     * Convert an angle and radius to a linear distance. This is the arc length formula
     * 
     * @param angle
     * @param radius
     * @return
     */
    public Distance toDistance(Angle angle)
    {
        return rollerRadius.times(angle.in(Radians));
    }

    /**
     * Convert a linear distance and radius to an angle. This is the arc length formula
     * 
     * @param position
     * @param radius
     * @return
     */
    public Angle toAngle(Distance position)
    {
        return Radians.of(position.div(rollerRadius).magnitude());
    }

    /**
     * Convert an angular velocity and radius to a linear velocity.
     * 
     * @param angularVelocity
     * @param radius
     * @return
     */
    public LinearVelocity toLinearVelocity(AngularVelocity angularVelocity)
    {
        return MetersPerSecond.of(angularVelocity.in(RadiansPerSecond) * rollerRadius.in(Meters));
    }

    /**
     * Convert a linear velocity and radius to an angular velocity.
     * 
     * @param linearVelocity
     * @param radius
     * @return
     */
    public AngularVelocity toAngularVelocity(LinearVelocity linearVelocity)
    {
        return RadiansPerSecond.of(linearVelocity.in(MetersPerSecond) / rollerRadius.in(Meters));
    }

    /**
     * Convert a linear velocity and radius to an angular velocity.
     * 
     * @param linearVelocity
     * @param radius
     * @return
     */
    public AngularAcceleration toAngularAcceleration(LinearAcceleration linearAccel)
    {
        return RadiansPerSecondPerSecond.of(linearAccel.in(MetersPerSecondPerSecond) / rollerRadius.in(Meters));
    }

    /**
     * Convert a linear velocity and radius to an angular velocity.
     * 
     * @param linearVelocity
     * @param radius
     * @return
     */
    public LinearAcceleration toLinearAcceleration(AngularAcceleration angularAccel)
    {
        return MetersPerSecondPerSecond.of(angularAccel.in(RadiansPerSecondPerSecond) * rollerRadius.in(Meters));
    }

    /**
     * Convert a linear jerk and radius to an angular jerk.
     * 
     * @param linearJerk
     * @param radius
     * @return
     */
    public Velocity<AngularAccelerationUnit> toAngularJerk(Velocity<LinearAccelerationUnit> linearJerk)
    {
        return RadiansPerSecondPerSecond.per(Second).of(linearJerk.in(MetersPerSecondPerSecond.per(Second)) / rollerRadius.in(Meters));
    }

}