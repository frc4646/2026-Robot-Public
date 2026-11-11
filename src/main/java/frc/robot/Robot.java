// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.LoggedCommandScheduler;
import frc.lib.util.LoggedTracer;
import frc.robot.configuration.Constants;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    @SuppressWarnings("unused")
    private boolean hasBeenEnabled = false;

    @Override
    public void robotInit()
    {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY)
        {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // always log to usb for now
        Logger.addDataReceiver(new WPILOGWriter());

        if (DriverStation.isFMSAttached())
        {
            // Only turn on logs if we are attached to FMS, but disable NTtables.
            // Logger.addDataReceiver(new WPILOGWriter());
        }
        else
        {
            // Otherwise, enable NT tables.
            Logger.addDataReceiver(new NT4Publisher());
        }

        if (Constants.IS_REPLAY)
        {
            String replayLogPath = LogFileUtil.findReplayLog();

            Logger.setReplaySource(new WPILOGReader(replayLogPath));
        }
        if (Constants.IS_REPLAY)
        {
            setUseTiming(true);
        }

        AutoLogOutputManager.addPackage("frc.lib");
        LoggedPowerDistribution.getInstance();

        LoggedCommandScheduler.init(CommandScheduler.getInstance());

        // Start the logger
        Logger.start();

        // Disable joystick console warnings
        DriverStation.silenceJoystickConnectionWarning(true);

        // Instantiate the robot
        robotContainer = new RobotContainer();

        // Put the command scheduler on the dashboard for logging what commands are running
        // SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    }

    @Override
    public void robotPeriodic()
    {
        LoggedTracer.reset();

        // Set this to max priority to ensure it doesn't get interrupted
        Threads.setCurrentThreadPriority(true, 99);

        double periodicTime = Timer.getTimestamp();
        // Scheduler runs Subsystem.periodic() methods prior to Command.init() or Command.execute()
        CommandScheduler.getInstance().run();
        Logger.recordOutput("LoggedRobot/CmdSchedMS", (Timer.getTimestamp() - periodicTime) * 1000.0);

        periodicTime = Timer.getTimestamp();
        robotContainer.updateOutputs();
        Logger.recordOutput("LoggedRobot/UpdateOutputsMS", (Timer.getTimestamp() - periodicTime) * 1000.0);

        LoggedCommandScheduler.periodic();

        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);
    }

    @Override
    public void disabledInit()
    {
        robotContainer.onDisable();
    }

    @Override
    public void disabledPeriodic()
    {
        robotContainer.disabledPeriodic();

        autonomousCommand = robotContainer.getAutonomousCommand();
    }

    @Override
    public void disabledExit()
    {
    }

    @Override
    public void autonomousInit()
    {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null)
        {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
        robotContainer.onEnable(true);

        hasBeenEnabled = true;
    }

    @Override
    public void autonomousPeriodic()
    {
    }

    @Override
    public void autonomousExit()
    {
    }

    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
        robotContainer.onEnable(false);

        hasBeenEnabled = true;
    }

    @Override
    public void teleopPeriodic()
    {
    }

    @Override
    public void teleopExit()
    {
    }

    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic()
    {
    }

    @Override
    public void testExit()
    {
    }

    @Override
    public void simulationPeriodic()
    {
    }
}
