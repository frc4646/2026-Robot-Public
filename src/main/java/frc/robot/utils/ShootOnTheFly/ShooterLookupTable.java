// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.ShootOnTheFly;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.WildcardInterpolatingTreeMap;

/** Add your docs here. */
public class ShooterLookupTable
{
    public static record ShooterParameters(AngularVelocity rpm, Angle angle, double timeOfFlight)
    {
    }

    private final WildcardInterpolatingTreeMap<Distance, ShooterParameters> map;

    public ShooterLookupTable()
    {
        map = new WildcardInterpolatingTreeMap<>(
                distanceInverseInterpolator(),
                shooterParametersInterpolator(),
                (d1, d2) -> Double.compare(d1.in(Meters), d2.in(Meters)));
    }

    private static InverseInterpolator<Distance> distanceInverseInterpolator()
    {
        return (start, end, query) ->
        {
            double totalRange = end.in(Meters) - start.in(Meters);
            if (totalRange == 0)
            {
                return 0.0;
            }
            return (query.in(Meters) - start.in(Meters)) / totalRange;
        };
    }

    private static Interpolator<ShooterParameters> shooterParametersInterpolator()
    {
        return (start, end, t) ->
        {
            double rpm = MathUtil.interpolate(start.rpm().in(RPM), end.rpm().in(RPM), t);
            double angle = MathUtil.interpolate(start.angle().in(Degrees), end.angle().in(Degrees), t);
            double timeOfFlight = MathUtil.interpolate(start.timeOfFlight(), end.timeOfFlight(), t);
            return new ShooterParameters(RPM.of(rpm), Degrees.of(angle), timeOfFlight);
        };
    }

    public void put(Distance distance, ShooterParameters params)
    {
        map.put(distance, params);
    }

    public ShooterParameters get(Distance distance)
    {
        return map.get(distance);
    }

    // TODO: what to use for entryset and lastKey
    public double velocityToEffectiveDistance(double velocity)
    {
        // Binary search or iterate through table to find distance
        // where (distance / ToF) = velocity
        // Most InterpolatingTreeMap implementations support inverse lookup
        // or you can build a reverse map: velocity → distance

        for (Map.Entry<Distance, ShooterParameters> entry : map.entrySet())
        {
            double dist = entry.getKey().in(Meters);
            double vel = dist / entry.getValue().timeOfFlight;
            if (vel >= velocity)
            {
                return dist; // Interpolate for better accuracy
            }
        }
        return map.lastKey().in(Meters); // Clamp to max
    }

}
