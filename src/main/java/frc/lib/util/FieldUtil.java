package frc.lib.util;

import static edu.wpi.first.units.Units.Meters;

import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.utils.FieldConstants_2026;

public class FieldUtil
{

    public static double fieldWidth = FieldConstants_2026.fieldWidth;
    public static double fieldLength = FieldConstants_2026.fieldLength;

    // TODO Find a way to not have duplicate constants
    public static Distance FIELD_WIDTH = Meters.of(fieldWidth);
    public static Distance FIELD_LENGTH = Meters.of(fieldLength);

    public static boolean shouldFlip()
    {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public static double applyX(double x_m)
    {
        return shouldFlip() ? fieldLength - x_m : x_m;
    }

    public static double applyY(double y_m)
    {
        return shouldFlip() ? fieldWidth - y_m : y_m;
    }

    public static Translation2d apply(Translation2d translation)
    {
        return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
    }

    public static Rotation2d apply(Rotation2d rotation)
    {
        return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
    }

    public static Pose2d apply(Pose2d pose)
    {
        return shouldFlip()
                ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
                : pose;
    }

    public static boolean inZone(Pose2d point, Pose2d[] points)
    {
        boolean result = false;
        if (point == null)
        {
            return false;
        }
        for (int i = 0, j = points.length - 1; i < points.length; j = i++)
        {
            if ((points[i].getY() > point.getY()) != (points[j].getY() > point.getY()) &&
                    (point.getX() < (points[j].getX() - points[i].getX()) * (point.getY() - points[i].getY()) / (points[j].getY() - points[i].getY())
                            + points[i].getX()))
            {
                result = !result;
            }
        }
        return result;
    }

    /***
     * Get distance to a point. Flips point if red.
     * 
     * @param point
     * @return
     */
    public static double getDistanceToPoint(Pose2d currentPose, Pose2d point)
    {
        Transform2d coord = currentPose.minus(point);
        return coord.getTranslation().getNorm();
    }

    public static double mirrorAndGetDistance(Pose2d currentPose, Pose2d point)
    {
        point = flipFieldPointIfRed(point);
        return getDistanceToPoint(currentPose, point);
    }

    public static Pose2d flipFieldPointIfRed(Pose2d point)
    {
        if (!RobotContainer.isOnRedAlliance())
        {
            return point;
        }
        return new Pose2d(FIELD_LENGTH.minus(Meters.of(point.getX())), Meters.of(point.getY()), point.getRotation());
    }

    public static boolean pointInPolygon(Point point, ArrayList<Point> polygon)
    {
        Path2D path = new Path2D.Double();

        // Move to the first point in the polygon
        path.moveTo(polygon.get(0).x, polygon.get(0).y);

        // Connect the points in the polygon
        for (int i = 1; i < polygon.size(); i++)
        {
            path.lineTo(polygon.get(i).x, polygon.get(i).y);
        }

        // Close the path
        path.closePath();

        // Create a Point2D object for the test point
        Point2D testPoint = new Point2D.Double(point.x, point.y);

        // Check if the test point is inside the polygon
        return path.contains(testPoint);
    }

    /***
     * Get angle for rotation to a point. Flips point if red.
     * 
     * @param point
     * @param flipBot
     *            if true, points using the back of the robot (180 deg offset)
     * @return
     */
    public static Rotation2d getAngleToPoint(Pose2d currentPose, Pose2d point, boolean flipBot)
    {
        point = apply(point);
        Rotation2d theta = new Rotation2d(point.getX() - currentPose.getX(), point.getY() - currentPose.getY());

        if (flipBot)
        {
            theta = theta.rotateBy(Rotation2d.fromDegrees(180.0));
        }
        return theta;
    }

    public static boolean isInFieldBoundary(Pose2d pose)
    {
        // this would be reusable year to year if we just change the field boundary constant
        return inZone(pose, FieldConstants_2026.FieldBoundary);
    }
}
