// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.ShootOnTheFly;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.util.FieldUtil;
import frc.robot.configuration.Constants;
import frc.robot.utils.FieldConstants_2026;
import frc.robot.utils.ShootOnTheFly.ShooterLookupTable.ShooterParameters;

/** Add your docs here. */
public class ShooterController
{
    public static record ShotValues(AngularVelocity rpm, Angle hoodAngle, Angle TurretAngle)
    {
    }

    private static String name = "ShooterController";

    public static ShotValues calculate(
            Translation2d robotPosition,
            Translation2d robotVelocity,
            Translation2d goalPosition,
            double latencyCompensation)
    {
        // 1. Project future position

        Logger.recordOutput(name + "/goalPosition", goalPosition);
        Translation2d futurePos = robotPosition.plus(
                robotVelocity.times(latencyCompensation));

        // 2. Get target vector
        Translation2d toGoal = goalPosition.minus(futurePos);
        double distance = toGoal.getNorm();
        Translation2d targetDirection = toGoal.div(distance);

        // 3. Look up baseline velocity from table
        ShooterParameters baseline;
        boolean feedMap;
        // if flip(hubPose).gettransaltion == targetpose
        if (FieldUtil.flipFieldPointIfRed(FieldConstants_2026.Hub.HubPose).getTranslation().minus(goalPosition).getNorm() < 1.0)
        {
            baseline = Constants.SHOOTER_MAP.get(Meters.of(distance));
            feedMap = false;
        }
        else
        {
            baseline = Constants.FEED_MAP.get(Meters.of(distance));
            feedMap = true;
        }
        // ShooterParameters baseline = Constants.SHOOTER_MAP.get(Meters.of(distance));
        // else
        // usef feed map

        Logger.recordOutput(name + "/baselineRPM", baseline.rpm());
        Logger.recordOutput(name + "/baselineAngle", baseline.angle());
        Logger.recordOutput(name + "/FeedMap", feedMap);
        double baselineVelocity = distance / baseline.timeOfFlight();

        // 4. Build target velocity vector
        Translation2d targetVelocity = targetDirection.times(baselineVelocity);

        // 5. THE MAGIC: subtract robot velocity
        Translation2d shotVelocityRotation = targetVelocity.minus(robotVelocity.times(Constants.SHOOT_ON_THE_FLY.ROBOT_VECTOR_ROTATION_FACTOR));
        Translation2d shotVelocity = targetVelocity.minus(robotVelocity.times(Constants.SHOOT_ON_THE_FLY.ROBOT_VECTOR_RPM_FACTOR));

        // 6. Extract results
        Rotation2d turretAngle = shotVelocityRotation.getAngle();
        double requiredVelocity = shotVelocity.getNorm();

        // 7. Use table in reverse: velocity → effective distance → RPM
        // TODO: figure out how to do the reverse lookup
        double effectiveDistance = Constants.SHOOTER_MAP.velocityToEffectiveDistance(requiredVelocity);
        AngularVelocity requiredRpm = Constants.SHOOTER_MAP.get(Meters.of(effectiveDistance)).rpm();

        double velocityRatio = requiredVelocity / baselineVelocity;

        // Split the correction: sqrt gives equal "contribution" from each
        double rpmFactor = Math.sqrt(velocityRatio);
        double hoodFactor = Math.sqrt(velocityRatio);

        // Apply RPM scaling
        double adjustedRpm = baseline.rpm().in(RPM) * rpmFactor;

        // Apply hood adjustment (changes horizontal component)
        double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.angle().in(Degrees)));
        double targetHorizFromHood = baselineVelocity * hoodFactor;
        double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
        double adjustedHood = Math.toDegrees(Math.acos(ratio));

        Logger.recordOutput(name + "/adjustedHood", adjustedHood);
        Logger.recordOutput(name + "/adjustedRpm", adjustedRpm);
        Logger.recordOutput(name + "/turretAngle", turretAngle.getMeasure());

        return new ShotValues(RPM.of(adjustedRpm), Degrees.of(adjustedHood), turretAngle.getMeasure());
    }
}