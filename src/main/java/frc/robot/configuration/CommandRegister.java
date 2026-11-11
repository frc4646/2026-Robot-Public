// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configuration;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.CommandFactory;
import frc.robot.commands.drive.DriveLock;

/** Add your docs here. */
public class CommandRegister
{
    // private final CommandFactory commandFactory;

    // NamedCommands.registerCommand("Coral Intake", commandFactory.CoralGroundIntake());

    public static void register(CommandFactory commandFactory)
    {
        NamedCommands.registerCommand("Shoot", commandFactory.shootCommand());

        NamedCommands.registerCommand("Intake", commandFactory.intakeCommand());

        NamedCommands.registerCommand("Climb", null);

        NamedCommands.registerCommand("IntakeJostle", commandFactory.jostleIntake());

        NamedCommands.registerCommand("EndWhenDoneShooting", commandFactory.endWhenDoneShooting());

        NamedCommands.registerCommand("EndWhenStartShooting", commandFactory.endWhenStartShooting());

        NamedCommands.registerCommand("StowAndRunIntake", commandFactory.stowAndRunIntakeCommand());

        NamedCommands.registerCommand("RevAndAim", commandFactory.revShooterAndAimCommand());

        NamedCommands.registerCommand("IntakeDeployUntilOut", commandFactory.intakeDeployUntilOut());

        NamedCommands.registerCommand("HoodDefaultCommand", commandFactory.turretHoodDefault());

        NamedCommands.registerCommand("IndexerDefaultCommand", commandFactory.indexerDefault());
        NamedCommands.registerCommand("DriveLock", new DriveLock());

        NamedCommands.registerCommand("ShootWhileLidOpen", commandFactory.shootWhileLidOpen());

        NamedCommands.registerCommand("IntakeWhileLidClosed", commandFactory.intakeWhileLidClosed());

        NamedCommands.registerCommand("WaitUntilLidClosed", commandFactory.waitUntilLidClosed());

        NamedCommands.registerCommand("WaitUntilLidOpen", commandFactory.waitUntillLidOpen());

        NamedCommands.registerCommand("WaitUntilIntakeDeployed", commandFactory.waitUntilIntakeDeploy());
    }
}
