// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldUtil;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.configuration.Constants;

public class DriveTeleop extends Command
{
    private final double kJoystickDeadband = .05; // Add a 5% deadband

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.SWERVE.MaxSpeed * kJoystickDeadband)
            .withRotationalDeadband(Constants.SWERVE.MaxAngularRate * kJoystickDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    FieldCentricFacingAngle pointAtTarget = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Constants.SWERVE.MaxSpeed * kJoystickDeadband)
            .withRotationalDeadband(Constants.SWERVE.MaxAngularRate * kJoystickDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final PhoenixPIDController turnPID = new PhoenixPIDController(
            Constants.SWERVE.PID_ROTATION_KP,
            Constants.SWERVE.PID_ROTATION_KI,
            Constants.SWERVE.PID_ROTATION_KD);

    private boolean headingCorrection = true;
    private double HEADING_CORRECTION_ROTATION_RATE_DEADBAND = 0.1;
    private double HEADING_CORRECTION_VELOCITY_DEADBAND = 0.5;

    private RobotState robotState;

    public DriveTeleop(RobotState robotState)
    {
        this.robotState = robotState;
        addRequirements(RobotContainer.SWERVE);
        pointAtTarget.HeadingController = turnPID;
        pointAtTarget.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize()
    {
        robotState.swerve.setLastRotation(FieldUtil.apply(RobotContainer.SWERVE.getState().Pose.getRotation()));
    }

    @Override
    public void execute()
    {
        double speedModifier = 1.0;

        double velocityX = RobotContainer.CONTROLS_DRIVE.getXSpeed() * speedModifier;
        double velocityY = RobotContainer.CONTROLS_DRIVE.getYSpeed() * speedModifier;
        double rotationalRate = RobotContainer.CONTROLS_DRIVE.getRotationRate();

        if (headingCorrection && // if moving but not rotating
                Math.abs(rotationalRate) < HEADING_CORRECTION_ROTATION_RATE_DEADBAND &&
                (Math.abs(velocityX) > HEADING_CORRECTION_VELOCITY_DEADBAND ||
                        Math.abs(velocityY) > HEADING_CORRECTION_VELOCITY_DEADBAND))
        {
            RobotContainer.SWERVE.setControl(
                    pointAtTarget
                            .withTargetDirection(robotState.swerve.getLastRotation())
                            .withRotationalDeadband(Constants.SWERVE.ROTATIONAL_RATE_DEADBAND)
                            .withVelocityX(velocityX)
                            .withVelocityY(velocityY));
        }
        else
        {
            RobotContainer.SWERVE.setControl(
                    drive
                            .withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(rotationalRate));
            robotState.swerve.setLastRotation(FieldUtil.apply(RobotContainer.SWERVE.getState().Pose.getRotation()));
        }
    }
}
