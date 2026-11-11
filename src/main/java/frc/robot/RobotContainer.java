// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.AutoModeSelector;
import frc.lib.servomotor.ServoMotorViz;
import frc.lib.vision.OrinVision;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.drive.DriveTeleop;
import frc.robot.configuration.CommandRegister;
import frc.robot.configuration.Constants;
import frc.robot.configuration.DriverControlsXbox;
import frc.robot.configuration.OperatorControls;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.IntakeArm.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollers.IntakeRollerSubsystem;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Swerve.Telemetry;
import frc.robot.subsystems.Swerve.TunerConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.TurretHood.TurretHoodSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    public static boolean isAuto;
    private final RobotState robotState;
    public static CommandSwerveDrivetrain SWERVE;
    private final LEDsSubsystem LEDS;

    private final Telemetry telemetry = new Telemetry(Constants.SWERVE.MaxSpeed);
    public static DriverControlsXbox CONTROLS_DRIVE;
    public static OperatorControls CONTROLS_OPERATOR;
    private final IntakeArmSubsystem INTAKE_ARM;
    private final IntakeRollerSubsystem INTAKE_ROLLERS;
    private final FlywheelSubsystem FLYWHEEL;
    private final TurretSubsystem TURRET;
    private final TurretHoodSubsystem TURRETHOOD;
    private final IndexerSubsystem INDEXER;
    @SuppressWarnings("unused")
    private final OrinVision VISION;

    // misc
    private static boolean mIsOnRedAlliance = false;
    public AutoModeSelector autoModeSelector;
    boolean lastAllianceWasRed = false;
    private final RobotCAD robotCAD;
    public static boolean enabled = false;

    private CommandFactory commandFactory;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        robotState = new RobotState();
        LEDS = new LEDsSubsystem(robotState);

        // Create Subsystems
        INDEXER = new IndexerSubsystem(robotState);
        INTAKE_ROLLERS = new IntakeRollerSubsystem(robotState);
        INTAKE_ARM = new IntakeArmSubsystem(robotState);
        FLYWHEEL = new FlywheelSubsystem(robotState);
        TURRETHOOD = new TurretHoodSubsystem(robotState);
        TURRET = new TurretSubsystem(robotState);
        SWERVE = TunerConstants.createDrivetrain(robotState);
        // CLIMBER = new ClimberSubsystem(robotState);
        VISION = new OrinVision(robotState, SWERVE);

        robotCAD = new RobotCAD(robotState);

        commandFactory = new CommandFactory(INDEXER, TURRETHOOD, FLYWHEEL, INTAKE_ROLLERS, INTAKE_ARM, TURRET, robotState, LEDS);

        // Create Controls
        CONTROLS_DRIVE = new DriverControlsXbox(commandFactory, robotState, SWERVE);
        CONTROLS_OPERATOR = new OperatorControls(commandFactory, robotState, SWERVE, this);

        // Bind Controls
        CONTROLS_OPERATOR.configureControls(INDEXER);
        // CONTROLS_OPERATOR.configureControls(INTAKE_ROLLERS);
        // CONTROLS_OPERATOR.configureControls(INTAKE_ARM);
        CONTROLS_OPERATOR.configureControls(FLYWHEEL);
        CONTROLS_OPERATOR.configureControls(TURRETHOOD);
        // CONTROLS_OPERATOR.configureControls(TURRET);

        CONTROLS_DRIVE.configureControls(INDEXER);
        CONTROLS_DRIVE.configureControls(INTAKE_ROLLERS);
        CONTROLS_DRIVE.configureControls(INTAKE_ARM);
        CONTROLS_DRIVE.configureControls(FLYWHEEL);
        CONTROLS_DRIVE.configureControls(TURRETHOOD);
        CONTROLS_DRIVE.configureControls(TURRET);

        CommandRegister.register(commandFactory);

        autoModeSelector = new AutoModeSelector();
        SWERVE.registerTelemetry(telemetry::telemeterize);
        SWERVE.setDefaultCommand(new DriveTeleop(robotState));

    }

    public RobotState getRobotState()
    {
        return robotState;
    }

    public void updateOutputs()
    {
        robotState.update();

        // Update CAD visualization with current robot pose
        robotCAD.update();

        ServoMotorViz.logRoot();
    }

    public void onEnable(boolean isAutonomous)
    {
        // set default commands
        enabled = true;

        if (isAutonomous)
        {
            // TODO set autonomous default commands
        }
        else
        {
            // TODO set teleop default commands
        }
        RobotContainer.isAuto = isAutonomous;
    }

    public void onDisable()
    {
        autoModeSelector.reset();
        autoModeSelector.update();

    }

    public Command getAutonomousCommand()
    {
        Command autonomousCommand;

        Optional<Command> autoMode = autoModeSelector.getAutoMode();
        if (autoMode.isPresent())
        {
            // System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
            autonomousCommand = autoMode.get();
        }
        else
        {
            System.out.println("NO AUTO MODE SET");
            autonomousCommand = new WaitCommand(1);
        }

        return autonomousCommand;
    }

    public static boolean isOnRedAlliance()
    {
        return mIsOnRedAlliance;
    }

    // new funct yto get autoModeSelector.getPathPreview()
    // return autoModeSelector.getPathPreview();

    public ArrayList<Pose2d> getAutoStart()
    {
        return autoModeSelector.getPathPreview();
    }

    public void disabledPeriodic()
    {
        var alliance = DriverStation.getAlliance();
        mIsOnRedAlliance = alliance.isPresent() && alliance.get() == Alliance.Red;

        // if auto changed, push the new path to dashboard
        if (autoModeSelector.update() || (lastAllianceWasRed != mIsOnRedAlliance))
        {

            telemetry.publishPath(autoModeSelector.getPathPreview());
            lastAllianceWasRed = mIsOnRedAlliance;
        }

        // run each subsystem's disabled code
    }

    public static BooleanSupplier isAutonomous()
    {
        return () -> isAuto;
    }
}
