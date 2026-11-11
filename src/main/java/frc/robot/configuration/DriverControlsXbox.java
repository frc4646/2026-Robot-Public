package frc.robot.configuration;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.FieldUtil;
import frc.robot.RobotState;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.drive.DrivePursue;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.IntakeArm.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollers.IntakeRollerSubsystem;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.TurretHood.TurretHoodSubsystem;
import frc.robot.utils.FieldUtil_2026;

public class DriverControlsXbox
{
    private double SLOW_MODE_MAX_PERCENT = 0.5;

    private final CommandXboxController controller = new CommandXboxController(0);

    private SlewRateLimiter xspeedLimiter;
    private SlewRateLimiter yspeedLimiter;
    private SlewRateLimiter rotLimiter;
    private Rotation2d previousPursuitRotation;

    private RobotState robotState;

    // CTRE's examples
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new
    // SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public DriverControlsXbox(CommandFactory commandFactory, RobotState robotstate, CommandSwerveDrivetrain swerve)
    {
        xspeedLimiter = new SlewRateLimiter(Constants.SWERVE.MaxSpeedAccel);
        yspeedLimiter = new SlewRateLimiter(Constants.SWERVE.MaxSpeedAccel);
        rotLimiter = new SlewRateLimiter(Constants.SWERVE.MaxAngularRateAccel);
        previousPursuitRotation = new Rotation2d();

        this.robotState = robotstate;

        // Reset Gyro
        controller.back()
                .onTrue(swerve
                        .runOnce(
                                () -> swerve.resetRotation(new Rotation2d(
                                        DriverStation.getAlliance().get() == Alliance.Red
                                                ? Degrees.of(-90)
                                                : Degrees.of(90))))
                        .ignoringDisable(true));

        controller.start().onTrue(
                swerve.runOnce(
                        () -> swerve.resetPose(FieldUtil.flipFieldPointIfRed(
                                new Pose2d(3.4, 4.0, new Rotation2d(Degrees.of(0))))))
                        .andThen(
                                swerve.runOnce(() -> swerve.resetRotation(new Rotation2d(
                                        DriverStation.getAlliance().get() == Alliance.Red
                                                ? Degrees.of(0)
                                                : Degrees.of(180)))))
                        .ignoringDisable(true));

        // controller.rightStick().whileTrue(
        // new DrivePointAtHub(robotstate));

        // controller.leftStick().whileTrue(
        // new DrivePointAtTarget(FieldUtil_2026.chooseTarget(robotstate), false)); // TODO: kp for rot is pretty low in sim,
        // increase it? test on actual
        // // bot

        controller.rightStick().whileTrue(new DrivePursue(robotstate));

        controller.rightBumper().whileTrue(
                commandFactory.shootCommand());
        // controller.rightStick().whileTrue(commandFactory.jostleIntake());
        controller.leftStick().whileTrue(commandFactory.outtakeCommand());
        controller.leftBumper().whileTrue(commandFactory.intakeCommand());

        controller.x().whileTrue(commandFactory.intakeWhileLidClosed());
        controller.y().whileTrue(commandFactory.shootWhileLidOpen());

    }

    public void configureControls(IntakeArmSubsystem intakeArmSubsystem)
    {
        // controller.leftBumper().whileTrue(intakeArmSubsystem.deploy());
        controller.leftTrigger().whileTrue(intakeArmSubsystem.undeploy());

        // controller.povUp().whileTrue(intakeArmSubsystem.TestDeploy());
        // controller.povDown().whileTrue(intakeArmSubsystem.TestStow());

    }

    public void configureControls(IntakeRollerSubsystem intakeRollers)
    {
        // controller.leftBumper().whileTrue(intakeRollers.spinCommand()); /////////// intake
        controller.leftTrigger().whileTrue(intakeRollers.stopCommand());
    }

    public void configureControls(FlywheelSubsystem flywheel)
    {
        // controller.x().whileTrue(flywheel.setTargetRPM(() -> flywheel.startRPM));
        // controller.povRight().toggleOnTrue(flywheel.runOnce(() -> flywheel.addRPM(100)).ignoringDisable(true));
        // controller.povLeft().toggleOnTrue(flywheel.runOnce(() -> flywheel.addRPM(-100)).ignoringDisable(true));
    }

    public void configureControls(IndexerSubsystem indexer)
    {
        // controller.a().whileTrue(indexer.dashboardSetSpeedIndexer());
        // controller.a().whileTrue(indexer.spinSpindexerWITHfeeder(-0.5));

        controller.b().whileTrue(indexer.unIndex());
    }

    public void configureControls(TurretHoodSubsystem turretHood)
    {
        // controller.x().whileTrue(turretHood.getServoCommands().setTargetAngle(() -> turretHood.customAngle));
        // controller.povUp().toggleOnTrue(turretHood.runOnce(() -> turretHood.offsetDeg(Degrees.of(1))));
        // controller.povDown().toggleOnTrue(turretHood.runOnce(() -> turretHood.offsetDeg(Degrees.of(-1))));

        // controller.povLeft().toggleOnTrue(turretHood.forward15());
        // controller.povRight().toggleOnTrue(turretHood.forward5());
        // controller.povDown().toggleOnTrue(turretHood.forward0());
    }

    public void configureControls(TurretSubsystem turret)
    {
        // controller.y().whileTrue(turret.pointToHubCommand());
        // controller.y().whileTrue(turret.setAngle(() -> Degrees.of(0)));
        // controller.b().whileTrue(turret.setAngle(() -> Degrees.of(100)));
        // controller.a().whileTrue(turret.setAngle(() -> Degrees.of(-100)));
    }

    public double getXSpeed()
    {
        // Drive left with -X (left)
        double value = -controller.getLeftY();
        double limiter = controller.getRightTriggerAxis() * value * (1.0 - SLOW_MODE_MAX_PERCENT);
        value -= limiter;
        value *= Math.abs(value);

        value *= Constants.SWERVE.MaxSpeed;
        if (robotState.swerve.getShooting())
        {
            if (FieldUtil_2026.isInAllianceZone(robotState))
            {
                if (value > Constants.SWERVE.MAX_SHOOTING_SWERVE_SPEED)
                {
                    value = Constants.SWERVE.MAX_SHOOTING_SWERVE_SPEED;
                }
                else if (value < -Constants.SWERVE.MAX_SHOOTING_SWERVE_SPEED)
                {
                    value = -Constants.SWERVE.MAX_SHOOTING_SWERVE_SPEED;
                }
            }
        }
        value = xspeedLimiter.calculate(value);
        Logger.recordOutput("X-Speed", value);
        return value;
    }

    public double getYSpeed()
    {
        // Drive forward with -Y (forward)
        double value = -controller.getLeftX();
        double limiter = controller.getRightTriggerAxis() * value * (1.0 - SLOW_MODE_MAX_PERCENT);
        value -= limiter;
        value *= Math.abs(value);

        value *= Constants.SWERVE.MaxSpeed;
        if (robotState.swerve.getShooting())
        {
            if (FieldUtil_2026.isInAllianceZone(robotState))
            {
                if (value > Constants.SWERVE.MAX_SHOOTING_SWERVE_SPEED)
                {
                    value = Constants.SWERVE.MAX_SHOOTING_SWERVE_SPEED;
                }
                else if (value < -Constants.SWERVE.MAX_SHOOTING_SWERVE_SPEED)
                {
                    value = -Constants.SWERVE.MAX_SHOOTING_SWERVE_SPEED;
                }
            }
        }
        value = yspeedLimiter.calculate(value);
        Logger.recordOutput("Y-Speed", value);
        return value;
    }

    public double getRotationRate()
    {
        // Spin ccw with -X (left)
        double value = -controller.getRightX();
        double limiter = controller.getRightTriggerAxis() * value * (1.0 - SLOW_MODE_MAX_PERCENT);
        value -= limiter;
        value *= Math.abs(value);

        value *= Constants.SWERVE.MaxAngularRate;
        value = rotLimiter.calculate(value);
        return value;
    }

    public Rotation2d getPursuitDirection()
    {

        if (Math.abs(controller.getLeftY()) < 0.1 && Math.abs(controller.getLeftX()) < 0.1)
        {
            return previousPursuitRotation;
        }
        previousPursuitRotation = new Rotation2d(-controller.getLeftX(), controller.getLeftY());
        return previousPursuitRotation;
    }

    public int getSnapAngle()
    {
        return -1;
    }

    public Command rumbleCommand(double intensity, double seconds)
    {
        return Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, intensity))
                .andThen(new WaitCommand(seconds))
                .finallyDo(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0));
    }

}
