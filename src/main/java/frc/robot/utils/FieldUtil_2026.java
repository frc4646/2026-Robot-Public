//-----------------TODO: UPDATE WHEN MA RELEASES FIELD STUFF---------------------------

// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.FieldUtil;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

public class FieldUtil_2026
{
    public static Distance distanceToTarget(RobotState robotState, Pose2d targetPose)
    {
        return Meters.of(robotState.getTurretPose()
                .minus(FieldUtil.flipFieldPointIfRed(targetPose)).getTranslation().getNorm());
    }

    public static Rotation2d rotationToTarget(RobotState robotState, Pose2d targetPose)
    {
        Rotation2d swerveToHubRotation = FieldUtil.flipFieldPointIfRed(targetPose)
                .minus(robotState.getTurretPose()).getTranslation().getAngle();
        Rotation2d robotToHubRotation = robotState.getTurretPose().getTranslation().getAngle();
        Logger.recordOutput("robotState/swerveToHubRotation", swerveToHubRotation);
        Logger.recordOutput("robotState/robotToHubRotation", robotToHubRotation);
        Logger.recordOutput("robotState/inMeters", Inches.of(651.2).in(Meters) - 4.6);
        Logger.recordOutput("robotState/inInches", Inches.of(651.2));

        return robotToHubRotation.plus(swerveToHubRotation);
    }

    public static Rotation2d absoluteRotationToTarget(RobotState robotState, Pose2d targetPose)
    {
        if (targetPose == null || robotState.swerve.getRobotPose() == null)
        {
            return new Rotation2d(Degrees.of(0));
        }
        // Logger.recordOutput("hub pose", FieldUtil.flipFieldPointIfRed(targetPose));
        Pose2d turretPoseWithoutRotation = new Pose2d(robotState.getTurretPose().getX(),
                robotState.getTurretPose().getY(), new Rotation2d());
        return FieldUtil.flipFieldPointIfRed(targetPose).minus(turretPoseWithoutRotation).getTranslation().getAngle();
    }

    public static boolean isInAllianceZone(RobotState robotState)
    {
        if (robotState.swerve.getRobotPose() != null)
        {
            double swerveX = robotState.swerve.getRobotPose().getX();
            return !RobotContainer.isOnRedAlliance() && swerveX < FieldConstants_2026.LinesVertical.allianceZone ||
                    RobotContainer.isOnRedAlliance() && swerveX > FieldConstants_2026.LinesVertical.oppAllianceZone;
        }
        return false;
    }

    public static boolean isBehindHub(RobotState robotState)
    {
        if (robotState.getTurretPose() != null) 
        {
            double swerveY = robotState.swerve.getRobotPose().getY();
            double swerveX = robotState.swerve.getRobotPose().getX();
            if (RobotContainer.isOnRedAlliance()){
                return swerveY < (FieldConstants_2026.LinesHorizontal.leftBumpEnd - FieldConstants_2026.behindHubLenience) // ~4.60
                    && swerveY > (FieldConstants_2026.LinesHorizontal.rightBumpStart - FieldConstants_2026.behindHubLenience) // ~3.40
                    && !(swerveX < FieldConstants_2026.LinesVertical.center)
                    // If behind our hub (till the center) or behind oppHub
                    && !isInAllianceZone(robotState);
            }
            else{
                return swerveY < (FieldConstants_2026.LinesHorizontal.leftBumpEnd - FieldConstants_2026.behindHubLenience) // ~4.60
                    && swerveY > (FieldConstants_2026.LinesHorizontal.rightBumpStart - FieldConstants_2026.behindHubLenience) // ~3.40
                    && (swerveX < FieldConstants_2026.LinesVertical.center)
                    // If behind our hub (till the center) or behind oppHub
                    && !isInAllianceZone(robotState);
                    // && ; // only check 2/3 of the way through the nz
            }
        }
        return false;
    }

    public static boolean isBehindTower(RobotState robotState)
    {
        double swerveX = robotState.swerve.getRobotPose().getX();
        double swerveY = robotState.swerve.getRobotPose().getY();
        if (isInAllianceZone(robotState))
        {
            if (RobotContainer.isOnRedAlliance())
            {
                if (swerveX < 1.1 && (3.2 < swerveY && swerveY < 4.3))
                {
                    return true;
                }
            }
            else
            {
                if (swerveX > 15.44 && (3.2 < swerveY && swerveY < 4.3))
                {
                    return true;
                }
            }
        }
        return false;
    }

    public static Pose2d chooseTarget(RobotState robotState)
    {
        Pose2d returnPos;
        if (isInAllianceZone(robotState))
        {
            returnPos = FieldConstants_2026.Hub.HubPose;
        }
        else if (robotState.getTurretPose().getY() > FieldConstants_2026.LinesHorizontal.center)
        {
            if (robotState.getTurretPose().getX() > FieldConstants_2026.LinesVertical.oppAllianceZone)
            {
                returnPos = FieldConstants_2026.AllianceZone.leftStockpileTargetOpp;
            }
            else
            {
                returnPos = FieldConstants_2026.AllianceZone.leftStockpileTarget;
            }
        }
        else
        {
            if (robotState.getTurretPose().getX() > FieldConstants_2026.LinesVertical.oppAllianceZone)
            {
                returnPos = FieldConstants_2026.AllianceZone.rightStockpileTargetOpp;
            }
            else
            {
                returnPos = FieldConstants_2026.AllianceZone.rightStockpileTarget;
            }
        }

        return returnPos;

    }
}
