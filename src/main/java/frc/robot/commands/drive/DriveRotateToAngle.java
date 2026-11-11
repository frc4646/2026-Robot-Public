package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldUtil;
import frc.robot.RobotContainer;
import frc.robot.configuration.Constants;

public class DriveRotateToAngle extends Command
{

    private final PhoenixPIDController turnPID = new PhoenixPIDController(
            Constants.SWERVE.PID_ROTATION_KP,
            Constants.SWERVE.PID_ROTATION_KI,
            Constants.SWERVE.PID_ROTATION_KD);

    FieldCentricFacingAngle pointAtTarget = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity);
    private DoubleSupplier desiredAngleDegrees;
    private boolean flipFieldPointIfRed;

    public DriveRotateToAngle(DoubleSupplier angleInDegrees, boolean flipFieldPointIfRed)
    {
        desiredAngleDegrees = angleInDegrees;
        this.flipFieldPointIfRed = flipFieldPointIfRed;
        addRequirements(RobotContainer.SWERVE);
        pointAtTarget.HeadingController = turnPID;
        pointAtTarget.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public DriveRotateToAngle(DoubleSupplier angleInDegrees)
    {
        this(angleInDegrees, false);
    }

    @Override
    public void execute()
    {
        double desiredAngle = desiredAngleDegrees.getAsDouble();
        if (flipFieldPointIfRed)
        {
            desiredAngle = FieldUtil.apply(Rotation2d.fromDegrees(desiredAngle)).getDegrees();
        }
        Logger.recordOutput("PointToAngle", Degrees.of(desiredAngle).in(Radians));
        RobotContainer.SWERVE.setControl(pointAtTarget
                .withTargetDirection(Rotation2d.fromDegrees(desiredAngle))
                .withRotationalDeadband(Constants.SWERVE.ROTATIONAL_RATE_DEADBAND)
                .withVelocityX(RobotContainer.CONTROLS_DRIVE.getXSpeed())
                .withVelocityY(RobotContainer.CONTROLS_DRIVE.getYSpeed()));
    }
}
