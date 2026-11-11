// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import frc.robot.RobotContainer;
import frc.robot.configuration.Constants;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveGoToLocation extends Command
{

    private final double X_Y_ACTIVATION_DISTANCE_M = 15;

    private final PhoenixPIDController turnPID = new PhoenixPIDController(
            Constants.SWERVE.PID_ROTATION_KP,
            Constants.SWERVE.PID_ROTATION_KI,
            Constants.SWERVE.PID_ROTATION_KD);
    private final PhoenixPIDController driveX_PID = new PhoenixPIDController(
            Constants.SWERVE.PID_TRANSLATION_KP,
            Constants.SWERVE.PID_TRANSLATION_KI,
            Constants.SWERVE.PID_TRANSLATION_KD);
    private final PhoenixPIDController driveY_PID = new PhoenixPIDController(
            Constants.SWERVE.PID_TRANSLATION_KP,
            Constants.SWERVE.PID_TRANSLATION_KI,
            Constants.SWERVE.PID_TRANSLATION_KD);

    FieldCentricFacingAngle pointAtTarget = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity);

    Supplier<Pose2d> targetPose;
    Pose2d finalTargetPose;
    boolean overrideX, overrideY;
    double velocityMultiplier;

    public DriveGoToLocation(Supplier<Pose2d> targetPose, boolean overrideX,
            boolean overrideY)
    {
        this(targetPose, overrideX, overrideY, 1.0);
    }

    /**
     * 
     * @param targetPose
     * @param overrideX
     *            true to use controller inputs
     * @param overrideY
     *            true to use controller inputs
     */
    public DriveGoToLocation(Supplier<Pose2d> targetPose, boolean overrideX,
            boolean overrideY, double velocityMultiplier)
    {
        this.targetPose = targetPose;
        this.overrideX = overrideX;
        this.overrideY = overrideY;
        this.velocityMultiplier = velocityMultiplier;

        addRequirements(RobotContainer.SWERVE);
        pointAtTarget.HeadingController = turnPID;
        pointAtTarget.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        finalTargetPose = targetPose.get();

        if (finalTargetPose == null)
        {
            return;
        }

        // Pose2d tmpPoint = FieldUtil.flipFieldPointIfRed(new Pose2d(x.getAsDouble(), y.getAsDouble(), new Rotation2d()));
        SwerveDriveState state = RobotContainer.SWERVE.getState();
        Pose2d swervePose = state.Pose;

        double xVelocity = driveX_PID.calculate(swervePose.getX(), finalTargetPose.getX(), state.Timestamp);
        double yVelocity = driveY_PID.calculate(swervePose.getY(), finalTargetPose.getY(), state.Timestamp);

        if (RobotContainer.isOnRedAlliance())
        {
            xVelocity *= -1;
            yVelocity *= -1;
        }
        // use direct drive
        // xVelocity = driveX_PID.getPositionError() * 2;
        // yVelocity = driveY_PID.getPositionError() * 2;

        // cap them at the max swerve velocity
        xVelocity = MathUtil.clamp(xVelocity, -Constants.SWERVE.MaxSpeed, Constants.SWERVE.MaxSpeed);
        yVelocity = MathUtil.clamp(yVelocity, -Constants.SWERVE.MaxSpeed, Constants.SWERVE.MaxSpeed);

        // use the controller input if we want to have override control or if our
        // distance is too large
        if (overrideX || Math.abs(driveX_PID.getPositionError()) > X_Y_ACTIVATION_DISTANCE_M)
        {
            // xVelocity = RobotContainer.CONTROLS_DRIVE.getXSpeed();
        }
        if (overrideY || Math.abs(driveY_PID.getPositionError()) > X_Y_ACTIVATION_DISTANCE_M)
        {
            // yVelocity = RobotContainer.CONTROLS_DRIVE.getYSpeed();
        }

        Rotation2d targetRot = finalTargetPose.getRotation();
        // if (Math.abs(targetRot.getDegrees() - swervePose.getRotation().getDegrees()) < 1.0)
        // {
        // targetRot = new Rotation2d(swervePose.getRotation().getDegrees());
        // }

        Logger.recordOutput("DriveGoToLocation/Rotation", finalTargetPose.getRotation().getDegrees());
        Logger.recordOutput("DriveGoToLocation/x", finalTargetPose.getX());
        Logger.recordOutput("DriveGoToLocation/y", finalTargetPose.getY());
        Logger.recordOutput("DriveGoToLocation/XDIFF", driveX_PID.getPositionError());
        Logger.recordOutput("DriveGoToLocation/YDIFF", driveY_PID.getPositionError());
        Logger.recordOutput("DriveGoToLocation/xVelocity", xVelocity);
        Logger.recordOutput("DriveGoToLocation/yVelocity", yVelocity);
        Logger.recordOutput("DriveGoToLocation/rotTarget", targetRot);

        RobotContainer.SWERVE.setControl(pointAtTarget
                .withRotationalDeadband(Constants.SWERVE.ROTATIONAL_RATE_DEADBAND)
                .withVelocityX(xVelocity * velocityMultiplier)
                .withVelocityY(yVelocity * velocityMultiplier)
                .withTargetDirection(targetRot));
    }
}
