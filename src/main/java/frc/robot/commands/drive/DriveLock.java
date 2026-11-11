package frc.robot.commands.drive;

import frc.robot.RobotContainer;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveLock extends Command
{
    SwerveRequest lockBrakes = new SwerveRequest.SwerveDriveBrake();

    public DriveLock()
    {
        addRequirements(RobotContainer.SWERVE);
    }

    @Override
    public void execute()
    {
        RobotContainer.SWERVE.setControl(lockBrakes);
    }
}
