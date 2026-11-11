package frc.robot.commands.drive;

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
import frc.robot.configuration.Constants;

public class DrivePointAtTarget extends Command
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
    boolean flip;

    public DrivePointAtTarget(Pose2d target, boolean flipped)
    {
        desiredTarget = target;
        flip = flipped;
        addRequirements(RobotContainer.SWERVE);
        pointAtTarget.HeadingController = turnPID;
        pointAtTarget.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute()
    {
        Rotation2d testRot = FieldUtil.getAngleToPoint(RobotContainer.SWERVE.getState().Pose,
                desiredTarget, flip != RobotContainer.isOnRedAlliance()); // red/blue should be handled opposite

        RobotContainer.SWERVE.setControl(pointAtTarget
                .withTargetDirection(testRot)
                .withRotationalDeadband(Constants.SWERVE.ROTATIONAL_RATE_DEADBAND)
                .withVelocityX(RobotContainer.CONTROLS_DRIVE.getXSpeed())
                .withVelocityY(RobotContainer.CONTROLS_DRIVE.getYSpeed()));
    }
}
