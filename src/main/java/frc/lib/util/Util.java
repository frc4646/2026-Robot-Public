package frc.lib.util;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Contains basic functions that are used often.
 * 
 * From Team 254 with additions
 */
public class Util
{
    public static final double kEpsilon = 1e-12;

    public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    public static final double kFieldWidthMeters = kAprilTagLayout.getFieldWidth(); // distance between field walls, 8.211m
    public static final double kFieldLengthMeters = kAprilTagLayout.getFieldLength(); // distance between driver station walls, 16.541m

    /**
     * Prevent this class from being instantiated.
     */
    protected Util()
    {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude)
    {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max)
    {
        return Math.min(max, Math.max(min, v));
    }

    public static int limit(int v, int min, int max)
    {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude)
    {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /**
     * Checks if the given input is within the range (min, max), both exclusive.
     */
    public static boolean inRange(double v, double min, double max)
    {
        return v > min && v < max;
    }

    public static double interpolate(double a, double b, double x)
    {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings)
    {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i)
        {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1)
            {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon)
    {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b)
    {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon)
    {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon)
    {
        boolean result = true;
        for (Double value_in : list)
        {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static double handleDeadband(double value, double deadband)
    {
        deadband = Math.abs(deadband);
        if (deadband == 1)
        {
            return 0;
        }
        double scaledValue = (value + (value < 0 ? deadband : -deadband)) / (1 - deadband);
        return (Math.abs(value) > Math.abs(deadband)) ? scaledValue : 0;
    }

    public static Translation3d flipRedBlue(Translation3d original)
    {
        return new Translation3d(kFieldLengthMeters - original.getX(), original.getY(), original.getZ());
    }

    public static <T> Supplier<T> memoizeByIteration(IntSupplier iteration, Supplier<T> delegate)
    {
        AtomicReference<T> value = new AtomicReference<>();
        AtomicInteger last_iteration = new AtomicInteger(-1);
        return () ->
        {
            int last = last_iteration.get();
            int now = iteration.getAsInt();
            if (last != now)
            {
                value.updateAndGet((cur) -> null);
            }
            T val = value.get();
            if (val == null)
            {
                val = value.updateAndGet(cur -> cur == null ? delegate.get() : cur);
                last_iteration.set(now);
            }
            return val;
        };
    }

    /**
     * Remap an input (min to max) to a new min and max
     */
    public static double map(double input, double in_min, double in_max, double out_min, double out_max)
    {
        return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * Correct for a joystick's deadband and also apply a x^power factor. If *value*
     * is below *deadband*, treat as 0
     * 
     * @param value
     *            input, -1-1
     * @param deadband
     *            ignore below this, scale from 0-1 above this
     * @param power
     *            value^power scaling
     * @return
     */
    public static double handleJoystick(double value, double deadband, int power)
    {
        double banded = handleDeadband(value, deadband);
        if (power % 2 == 0)
        {
            // odd, so have to account for flipping the sign when applying the power
            return Math.abs(Math.pow(banded, power)) * Math.signum(banded);
        }
        else
        {
            // even
            return Math.pow(banded, power);
        }
    }

    public static double round(double value, int decimals)
    {
        double shift = Math.pow(10.0, decimals);
        return Math.round(value * shift) / shift;
    }
}
