package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldUtil;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.configuration.Constants;
import frc.robot.utils.FieldUtil_2026;

public class DrivePointAtHub extends Command
{
    private final PhoenixPIDController turnPID = new PhoenixPIDController(
            Constants.SWERVE.PID_ROTATION_KP,
            Constants.SWERVE.PID_ROTATION_KI,
            Constants.SWERVE.PID_ROTATION_KD);
    // no way that works irl

    FieldCentricFacingAngle pointAtTarget = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    Pose2d desiredTarget;
    RobotState robotState;

    public DrivePointAtHub(RobotState robotState)
    {
        this.robotState = robotState;
        addRequirements(RobotContainer.SWERVE);
        pointAtTarget.HeadingController = turnPID;
        pointAtTarget.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute()
    {
        Rotation2d rotationToHub = FieldUtil.apply(new Rotation2d(robotState.calculateShotValues(
                FieldUtil_2026.chooseTarget(robotState), Constants.SHOOT_ON_THE_FLY.LATENCY_COMPENSATION).TurretAngle()
                .plus((Degrees.of(-90))))); // TODO: haven't tested this with new SOTF

        RobotContainer.SWERVE.setControl(pointAtTarget
                .withTargetDirection(rotationToHub)
                .withRotationalDeadband(Constants.SWERVE.ROTATIONAL_RATE_DEADBAND)
                .withVelocityX(RobotContainer.CONTROLS_DRIVE.getXSpeed())
                .withVelocityY(RobotContainer.CONTROLS_DRIVE.getYSpeed()));
    }
}
