package frc.robot.configuration;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.IntakeArm.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollers.IntakeRollerSubsystem;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.TurretHood.TurretHoodSubsystem;

public class OperatorControls
{
    private final CommandXboxController controller = new CommandXboxController(1);

    private final RobotState robotState;
    private final CommandFactory commandFactory;

    public OperatorControls(CommandFactory commandFactory, RobotState robotState, CommandSwerveDrivetrain swerve, RobotContainer robotContainer)
    {
        this.robotState = robotState;
        this.commandFactory = commandFactory;

        controller.back()
                .onTrue(swerve
                        .runOnce(
                                () -> swerve.resetRotation(new Rotation2d(
                                        DriverStation.getAlliance().get() == Alliance.Red
                                                ? Degrees.of(90)
                                                : Degrees.of(-90))))
                        .ignoringDisable(true));

        // controller.a().
        controller.a().and(() -> DriverStation.isDisabled())
                .whileTrue(swerve
                        .runOnce(() ->
                        {
                            var autoStart = robotContainer.getAutoStart();
                            if (!autoStart.isEmpty())
                            {
                                swerve.resetTranslation(autoStart.get(0).getTranslation());
                            }
                        })
                        .ignoringDisable(true));
                        

        controller.povDown()
                .onTrue(swerve
                        .runOnce(
                                () -> swerve.resetRotation(new Rotation2d(
                                        DriverStation.getAlliance().get() == Alliance.Red
                                                ? Degrees.of(0)
                                                : Degrees.of(180))))
                        .ignoringDisable(true));

        controller.povUp()
                .onTrue(swerve
                        .runOnce(
                                () -> swerve.resetRotation(new Rotation2d(
                                        DriverStation.getAlliance().get() == Alliance.Red
                                                ? Degrees.of(180)
                                                : Degrees.of(0))))
                        .ignoringDisable(true));

        controller.povLeft()
                .onTrue(swerve
                        .runOnce(
                                () -> swerve.resetRotation(new Rotation2d(
                                        DriverStation.getAlliance().get() == Alliance.Red
                                                ? Degrees.of(-90)
                                                : Degrees.of(90))))
                        .ignoringDisable(true));

        controller.povRight()
                .onTrue(swerve
                        .runOnce(
                                () -> swerve.resetRotation(new Rotation2d(
                                        DriverStation.getAlliance().get() == Alliance.Red
                                                ? Degrees.of(90)
                                                : Degrees.of(-90))))
                        .ignoringDisable(true));
    }

    public void configureSuperStructure()
    {
        // controller.x().whileTrue();

    }

    public void configureControls(IntakeArmSubsystem intakeArmSubsystem)
    {
        controller.leftBumper().whileTrue(intakeArmSubsystem.deploy());
        controller.leftTrigger().whileTrue(intakeArmSubsystem.undeploy());
    }

    public void configureControls(IntakeRollerSubsystem intakeRollers)
    {
        controller.leftBumper().whileTrue(intakeRollers.spinCommand()); /////////// intake
        controller.leftTrigger().whileTrue(intakeRollers.stopCommand());
    }

    public void configureControls(TurretSubsystem turret)
    {
        // controller.x().whileTrue(turret.goTo10());
        // controller.y().whileTrue(turret.goTo50());
        // controller.rightTrigger().whileTrue(turret.goToNegativeCommand());
    }

    public void configureControls(FlywheelSubsystem flywheel)
    {
        // controller.a().whileTrue(flywheel.setTargetRPM(() -> RPM.of(500)));
        // testing command
        // controller.y().whileTrue(flywheel.testerCommand());
        controller.x().whileTrue(flywheel.setTargetRPM(() -> flywheel.startRPM));
        // controller.rightBumper().whileTrue(flywheel.setTargetRPM(() -> flywheel.startRPM));
        // controller.rightBumper().whileTrue(flywheel.setRPMByActualDist());

        controller.povUp().toggleOnTrue(flywheel.runOnce(() -> flywheel.addRPM(100)));
        controller.povDown().toggleOnTrue(flywheel.runOnce(() -> flywheel.addRPM(-100)));

        // controller.rightBumper().whileTrue(flywheel.setTargetRPM(() -> RPM.of(FlywheelConstants.FlywheelRPMMap.get(5.0))));
        // // Constant movement accelerating
        // controller.b().whileTrue(flywheel.getServoCommands().setTargetDutyCycle(() -> 0.1));
        // controller.x().whileTrue(flywheel.getServoCommands().setTargetDutyCycle(() -> 0.5));
    }

    public void configureControls(TurretHoodSubsystem turretHood)
    {
        // controller.povUp().whileTrue(turretHood.getServoCommands().setTargetAngle(Degrees.of(15)));
        // controller.povDown().whileTrue(turretHood.getServoCommands().setTargetAngle(Degrees.of(5)));

        // controller.leftBumper().whileTrue(turretHood.backwardsTH());
        // controller.rightBumper().whileTrue(turretHood.forwardTH());
        controller.y().whileTrue(turretHood.getServoCommands().setTargetAngle(turretHood.customAngle));

        controller.povLeft().toggleOnTrue(turretHood.runOnce(() -> turretHood.offsetDeg(Degrees.of(0.5))));
        controller.povRight().toggleOnTrue(turretHood.runOnce(() -> turretHood.offsetDeg(Degrees.of(-0.5))));
    }

    public void configureControls(IndexerSubsystem indexer)
    {
        // controller.a().whileTrue(indexer.feederSpinCommand());
        // controller.b().whileTrue(indexer.spindexerSpinCommand());
        // controller.x().whileTrue(indexer.dashboardSetSpeedIndexer());
        // controller.y().whileTrue(indexer.feederDoesntHaveFuelSpinCommand());
        // controller.povUp().whileTrue(indexer.feederHasFuelSpinCommand());
        // controller.povDown().whileTrue(indexer.spinSpindexerWITHfeeder(.70));
        controller.leftBumper().whileTrue(indexer.dashboardSetSpeedIndexer());
        controller.a().whileTrue(indexer.spinSpindexerWITHfeeder(-0.5));
    }
}