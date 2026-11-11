package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DrivePointTranslate extends Command
{
    PointWheelsAt pointWheels = new SwerveRequest.PointWheelsAt();

    public DrivePointTranslate()
    {
        addRequirements(RobotContainer.SWERVE);
    }

    @Override
    public void execute()
    {
        RobotContainer.SWERVE.setControl(pointWheels
                .withModuleDirection(new Rotation2d(RobotContainer.CONTROLS_DRIVE.getXSpeed(), RobotContainer.CONTROLS_DRIVE.getYSpeed())));
    }
}
