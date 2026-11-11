// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldUtil;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.configuration.Constants;

public class DrivePursue extends Command
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

    private RobotState robotState;

    public DrivePursue(RobotState robotState)
    {
        addRequirements(RobotContainer.SWERVE);
        this.robotState = robotState;
        pointAtTarget.HeadingController = turnPID;
        pointAtTarget.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        double speedModifier = 1.0;

        double velocityX = RobotContainer.CONTROLS_DRIVE.getXSpeed() * speedModifier;
        double velocityY = RobotContainer.CONTROLS_DRIVE.getYSpeed() * speedModifier;

        RobotContainer.SWERVE.setControl(
                pointAtTarget
                        .withTargetDirection(RobotContainer.CONTROLS_DRIVE.getPursuitDirection().plus(new Rotation2d(Degrees.of(90))))
                        .withRotationalDeadband(Constants.SWERVE.ROTATIONAL_RATE_DEADBAND)
                        .withVelocityX(velocityX)
                        .withVelocityY(velocityY));
        robotState.swerve.setLastRotation(FieldUtil.apply(RobotContainer.SWERVE.getState().Pose.getRotation()));

    }
}
