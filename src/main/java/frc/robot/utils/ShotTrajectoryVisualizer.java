package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ShotTrajectoryVisualizer
{
    private static final double GRAVITY = 9.81; // m/s²
    private static final int NUM_POINTS = 50;

    /**
     * Calculate and log shot trajectory for AdvantageScope visualization
     * 
     * @param launchPosition
     *            Field-relative position where ball exits shooter (meters)
     * @param yawRadians
     *            Horizontal angle (field-relative, 0 = +X direction)
     * @param pitchRadians
     *            Vertical angle above horizontal (positive = up)
     * @param velocityMps
     *            Exit velocity in meters per second
     */
    public static void logTrajectory(
            String shotName,
            Translation3d launchPosition,
            double yawRadians,
            double pitchRadians,
            double velocityMps)
    {

        // Decompose velocity into components
        double vHorizontal = velocityMps * Math.cos(pitchRadians);
        double vZ = velocityMps * Math.sin(pitchRadians); // vertical
        double vX = vHorizontal * Math.cos(yawRadians); // field X
        double vY = vHorizontal * Math.sin(yawRadians); // field Y

        // Calculate flight time until ball hits ground (z = 0)
        // Using quadratic formula: z = z0 + vZ*t - 0.5*g*t²
        // Solve for t when z = 0
        double z0 = launchPosition.getZ();
        double discriminant = vZ * vZ + 2 * GRAVITY * z0;
        double totalTime = (vZ + Math.sqrt(Math.max(0, discriminant))) / GRAVITY;

        double dt = totalTime / NUM_POINTS;

        List<Pose3d> trajectory = new ArrayList<>();

        for (int i = 0; i <= NUM_POINTS; i++)
        {
            double t = i * dt;

            double x = launchPosition.getX() + vX * t;
            double y = launchPosition.getY() + vY * t;
            double z = launchPosition.getZ() + vZ * t - 0.5 * GRAVITY * t * t;

            if (z < 0)
                break;

            trajectory.add(new Pose3d(x, y, z, new Rotation3d()));
        }

        Logger.recordOutput(shotName, trajectory.toArray(new Pose3d[0]));
    }

    /** Clear the trajectory visualization */
    public static void clearTrajectory(String shotName)
    {
        Logger.recordOutput(shotName, new Pose3d[0]);
    }
}