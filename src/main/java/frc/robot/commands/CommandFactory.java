package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelConstants;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.IntakeArm.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollers.IntakeRollerSubsystem;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.TurretHood.TurretHoodSubsystem;

public class CommandFactory
{

    private final IndexerSubsystem indexer;
    private final TurretHoodSubsystem turretHood;
    private final FlywheelSubsystem flywheel;
    private final IntakeRollerSubsystem intakeRollers;
    private final IntakeArmSubsystem intakeArm;
    private final TurretSubsystem turret;
    private final RobotState robotState;
    private final LEDsSubsystem LEDS;
    private static Color color;

    public CommandFactory(IndexerSubsystem indexer, TurretHoodSubsystem turretHood, FlywheelSubsystem flywheel, IntakeRollerSubsystem intakeRollers,
            IntakeArmSubsystem intakeArm, TurretSubsystem turret, RobotState robotState, LEDsSubsystem LEDS)
    {
        this.indexer = indexer;
        this.turretHood = turretHood;
        this.flywheel = flywheel;
        this.turret = turret;
        this.intakeRollers = intakeRollers;
        this.intakeArm = intakeArm;
        this.robotState = robotState;
        this.LEDS = LEDS;

    }

    /*
     * A shooting command that waits for the flywheel to get up to speed,
     * and also allows the turrethood to move during this time.
     * Once the flywheel is up to speed, runs a parallel command group
     * that runs the subsystems flywehel, turrethood, and indexer.
     */
    public Command shootCommand()
    {
        return Commands.sequence(
                Commands.runOnce(() ->
                {
                    robotState.swerve.setShooting(true);
                }),
                Commands.deadline(
                        Commands.parallel(
                                flywheel.setRPMByActualDist().until(() -> flywheel.getErrorAngularVelocity().lte(FlywheelConstants.TOLERANCE)),
                                turret.pointToHubCommand().until(() -> turret.getErrorAngle().lte(TurretConstants.TOLERANCE))),
                        turretHood.setToHubAngle()),

                Commands.parallel(
                        flywheel.setRPMByActualDist(),
                        turretHood.setToHubAngle(),
                        turret.pointToHubCommand(),
                        // (index.until(jammed).andthen(unindex with timeout)).repeatedly
                        Commands.repeatingSequence(
                                indexer.indexIfCanShootAngularVelocity().until(indexer.isSpindexerJammed.or(indexer.caving)),
                                indexer.unIndex().withTimeout(.25).andThen(indexer.indexIfCanShootAngularVelocity().withTimeout(1.5)))))
                // new SwitchingCommand(indexer.isSpindexerJammed, indexer.unIndex(), indexer.indexIfCanShootAngularVelocity())))
                // indexer.indexIfCanShootAngularVelocity()))
                .finallyDo((() ->
                {
                    robotState.swerve.setShooting(false);
                }));
    }

    public Command revShooterAndAimCommand()
    {
        return Commands.parallel(
                flywheel.setRPMByActualDist(),
                turret.pointToHubCommand());
    }

    public Command intakeCommand()
    {
        return Commands.parallel(
                intakeArm.deploy(),
                intakeRollers.spinIfCanIntakeCommand());
    }

    public Command outtakeCommand()
    {
        return Commands.parallel(
                intakeArm.deploy(),
                intakeRollers.ejectCommand());
    }

    public Command stowAndRunIntakeCommand()
    {
        return Commands.parallel(
                intakeArm.undeploy(),
                intakeRollers.spinCommand());
    }

    public Command intakeDeployUntilOut()
    {
        return (intakeArm.deploy().until(() -> intakeArm.getErrorAngle().abs(Degrees) < 2)).withDeadline(new WaitCommand(0.75));
    }

    public Command jostleIntake()
    {
        Command jostle = Commands.repeatingSequence(
                intakeArm.jostleStow().until(() -> intakeArm.getErrorAngle().abs(Degrees) < (5.0)).withTimeout(0.5),
                intakeArm.jostleDeploy().until(() -> intakeArm.getErrorAngle().abs(Degrees) < (5.0)).withTimeout(0.5));

        return Commands.parallel(
                jostle,
                intakeRollers.spinCommand());
    }

    public Command endWhenDoneShooting()
    {
        return new WaitUntilCommand(() -> (robotState.timeSinceLastShot > IndexerConstants.SHOOTING_TIMEOUT)
                && (indexer.isLidClosed())
                && (!indexer.isSpindexerJammed.getAsBoolean()));
    }

    public Command endWhenStartShooting()
    {
        return new WaitUntilCommand(() -> (robotState.timeSinceLastShot < IndexerConstants.SHOOTING_TIMEOUT));
    }

    public Command turretHoodDefault()
    {
        return turretHood.getDefaultCommand();
    }

    public Command indexerDefault()
    {
        return indexer.indexAndFeedCommand(0, 0, 0);
    }

    public Command intakeWhileLidClosed()
    {
        return Commands.repeatingSequence(
                intakeRollers.spinIfCanIntakeCommand().onlyWhile(() -> indexer.isLidClosed()),
                intakeRollers.stopCommand().onlyWhile(() -> !indexer.isLidClosed()));
    }

    public Command shootWhileLidOpen()
    {
        return Commands.repeatingSequence(
                shootCommand().onlyWhile(() -> !indexer.isLidClosed()),
                (Commands.parallel(revShooterAndAimCommand(), indexerDefault(), turretHoodDefault())).onlyWhile(() -> indexer.isLidClosed()));
    }

    public Command waitUntilLidClosed()
    {
        return Commands.waitUntil(() -> (indexer.getLidClosedTime() > .5));
    }

    public Command waitUntillLidOpen()
    {
        return Commands.waitUntil(() -> (indexer.getLidOpenTime() > .5));
    }

    public Command waitUntilIntakeDeploy()
    {
        return Commands.waitUntil(() -> intakeArm.getErrorAngle().abs(Degrees) < 2);
    }
}
