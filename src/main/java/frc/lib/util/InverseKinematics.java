package frc.lib.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class InverseKinematics
{
    private final Distance length1;
    private final Distance length2;

    public InverseKinematics(Distance length1, Distance length2)
    {
        this.length1 = length1;
        this.length2 = length2;
    }

    public Pair<Angle, Angle> get(Distance x, Distance y, boolean invert)
    {
        return get(new Translation2d(x, y), invert);
    }

    /**
     * Given a 2D position, return the angles for the lower and upper arms to reach that position.
     * Uses inverse kinematics for a 2-link planar arm.
     * 
     * @param position
     *            The target position in meters.
     * @param invert
     *            If true, use the "elbow down" configuration; otherwise, use "elbow up".
     * @return A Pair containing the angles for the lower and upper arms in radians.
     */
    public Pair<Angle, Angle> get(Translation2d position, boolean invert)
    {
        var x = Meters.of(position.getX()).in(Meters);
        var y = Meters.of(position.getY()).in(Meters);

        var len1 = length1.in(Meters);
        var len2 = length2.in(Meters);

        // Calculate theta2 with proper elbow configuration
        var theta2 = Math.acos(((x * x) + (y * y) - (len1 * len1) - (len2 * len2)) / (2 * len1 * len2));

        theta2 *= invert ? -1 : 1;

        // Calculate theta1
        var theta1 = Math.atan2(y, x) - Math.atan2(len2 * Math.sin(theta2), len1 + len2 * Math.cos(theta2));

        return Pair.of(Radians.of(theta1), Radians.of(theta2));
    }
}
