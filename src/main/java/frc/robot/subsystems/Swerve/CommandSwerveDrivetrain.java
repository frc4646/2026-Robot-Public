package frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.Joules;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutEnergy;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.subsystems.Swerve.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem
{
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private RobotState robotState;

    /* Motor telemetry - energy tracking */
    private final MutEnergy[] driveMotorEnergy = new MutEnergy[4];
    private final MutEnergy[] steerMotorEnergy = new MutEnergy[4];
    private MutEnergy totalDrivetrainEnergy;
    private double lastTelemetryTimestamp = 0.0;

    /* Motor telemetry - cached signal references for efficient batch refresh */
    private StatusSignal<Voltage>[] driveVoltageSignals;
    private StatusSignal<Current>[] driveSupplyCurrentSignals;
    private StatusSignal<Current>[] driveStatorCurrentSignals;
    private StatusSignal<Temperature>[] driveTemperatureSignals;
    private StatusSignal<Voltage>[] steerVoltageSignals;
    private StatusSignal<Current>[] steerSupplyCurrentSignals;
    private StatusSignal<Current>[] steerStatorCurrentSignals;
    private StatusSignal<Temperature>[] steerTemperatureSignals;
    private BaseStatusSignal[] allTelemetrySignals;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output ->
                    {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants
     *            Drivetrain-wide constants for the swerve drive
     * @param modules
     *            Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            RobotState robotState,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation())
        {
            startSimThread();
        }
        configureAutoBuilder();
        this.robotState = robotState;
        initializeTelemetrySignals();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants
     *            Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency
     *            The frequency to run the odometry loop. If
     *            unspecified or set to 0 Hz, this is 250 Hz on
     *            CAN FD, and 100 Hz on CAN 2.0.
     * @param modules
     *            Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            RobotState robotState,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation())
        {
            startSimThread();
        }
        configureAutoBuilder();
        this.robotState = robotState;
        initializeTelemetrySignals();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants
     *            Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency
     *            The frequency to run the odometry loop. If
     *            unspecified or set to 0 Hz, this is 250 Hz on
     *            CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation
     *            The standard deviation for odometry calculation
     *            in the form [x, y, theta]ᵀ, with units in meters
     *            and radians
     * @param visionStandardDeviation
     *            The standard deviation for vision calculation
     *            in the form [x, y, theta]ᵀ, with units in meters
     *            and radians
     * @param modules
     *            Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            RobotState robotState,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation())
        {
            startSimThread();
        }
        configureAutoBuilder();
        this.robotState = robotState;
        initializeTelemetrySignals();
    }

    private void configureAutoBuilder()
    {
        try
        {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(12.5, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(8.5, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
        } catch (Exception ex)
        {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    @SuppressWarnings("unchecked")
    private void initializeTelemetrySignals()
    {
        var modules = getModules();
        int moduleCount = modules.length;

        // Initialize energy accumulators
        totalDrivetrainEnergy = Joules.zero().mutableCopy();
        for (int i = 0; i < moduleCount; i++)
        {
            driveMotorEnergy[i] = Joules.zero().mutableCopy();
            steerMotorEnergy[i] = Joules.zero().mutableCopy();
        }

        // Cache signal references
        driveVoltageSignals = new StatusSignal[moduleCount];
        driveSupplyCurrentSignals = new StatusSignal[moduleCount];
        driveStatorCurrentSignals = new StatusSignal[moduleCount];
        driveTemperatureSignals = new StatusSignal[moduleCount];
        steerVoltageSignals = new StatusSignal[moduleCount];
        steerSupplyCurrentSignals = new StatusSignal[moduleCount];
        steerStatorCurrentSignals = new StatusSignal[moduleCount];
        steerTemperatureSignals = new StatusSignal[moduleCount];

        List<BaseStatusSignal> signalList = new ArrayList<>();

        for (int i = 0; i < moduleCount; i++)
        {
            TalonFX driveMotor = modules[i].getDriveMotor();
            TalonFX steerMotor = modules[i].getSteerMotor();

            driveVoltageSignals[i] = driveMotor.getMotorVoltage();
            driveSupplyCurrentSignals[i] = driveMotor.getSupplyCurrent();
            driveStatorCurrentSignals[i] = driveMotor.getStatorCurrent();
            driveTemperatureSignals[i] = driveMotor.getDeviceTemp();

            steerVoltageSignals[i] = steerMotor.getMotorVoltage();
            steerSupplyCurrentSignals[i] = steerMotor.getSupplyCurrent();
            steerStatorCurrentSignals[i] = steerMotor.getStatorCurrent();
            steerTemperatureSignals[i] = steerMotor.getDeviceTemp();

            // Add all to list for batch refresh
            signalList.add(driveVoltageSignals[i]);
            signalList.add(driveSupplyCurrentSignals[i]);
            signalList.add(driveStatorCurrentSignals[i]);
            signalList.add(driveTemperatureSignals[i]);
            signalList.add(steerVoltageSignals[i]);
            signalList.add(steerSupplyCurrentSignals[i]);
            signalList.add(steerStatorCurrentSignals[i]);
            signalList.add(steerTemperatureSignals[i]);
        }

        allTelemetrySignals = signalList.toArray(new BaseStatusSignal[0]);

        // Set update frequency (50Hz is sufficient for telemetry, reduces CAN load)
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, allTelemetrySignals);
    }

    private void logMotorTelemetry()
    {
        double timestamp = Timer.getFPGATimestamp();
        double dt = timestamp - lastTelemetryTimestamp;
        lastTelemetryTimestamp = timestamp;

        // Refresh all signals in one batch call for efficiency
        BaseStatusSignal.refreshAll(allTelemetrySignals);

        var modules = getModules();

        // Aggregates
        double totalPowerWatts = 0.0;

        for (int i = 0; i < modules.length; i++)
        {
            String modulePrefix = "Swerve/Module" + i;

            // === Drive Motor Telemetry ===
            double driveVoltage = driveVoltageSignals[i].getValueAsDouble();
            double driveSupplyCurrent = driveSupplyCurrentSignals[i].getValueAsDouble();
            double driveStatorCurrent = driveStatorCurrentSignals[i].getValueAsDouble();
            double driveTemp = driveTemperatureSignals[i].getValueAsDouble();

            // Calculate power (watts) = |voltage * supply current| (absolute to always count up)
            double drivePowerWatts = Math.abs(driveVoltage * driveSupplyCurrent);

            // Accumulate energy (joules) with dt guard
            if (dt > 0 && dt < 0.1)
            {
                driveMotorEnergy[i].mut_plus(Watts.of(drivePowerWatts).times(Seconds.of(dt)));
            }

            // Log drive motor signals
            Logger.recordOutput(modulePrefix + "/Drive/VoltageVolts", driveVoltage);
            Logger.recordOutput(modulePrefix + "/Drive/SupplyCurrentAmps", driveSupplyCurrent);
            Logger.recordOutput(modulePrefix + "/Drive/StatorCurrentAmps", driveStatorCurrent);
            Logger.recordOutput(modulePrefix + "/Drive/PowerWatts", drivePowerWatts);
            Logger.recordOutput(modulePrefix + "/Drive/EnergyJoules", driveMotorEnergy[i]);
            Logger.recordOutput(modulePrefix + "/Drive/TemperatureCelsius", driveTemp);

            // === Steer Motor Telemetry ===
            double steerVoltage = steerVoltageSignals[i].getValueAsDouble();
            double steerSupplyCurrent = steerSupplyCurrentSignals[i].getValueAsDouble();
            double steerStatorCurrent = steerStatorCurrentSignals[i].getValueAsDouble();
            double steerTemp = steerTemperatureSignals[i].getValueAsDouble();

            // Calculate power (watts) = |voltage * supply current| (absolute to always count up)
            double steerPowerWatts = Math.abs(steerVoltage * steerSupplyCurrent);

            // Accumulate energy (joules)
            if (dt > 0 && dt < 0.1)
            {
                steerMotorEnergy[i].mut_plus(Watts.of(steerPowerWatts).times(Seconds.of(dt)));
            }

            // Log steer motor signals
            Logger.recordOutput(modulePrefix + "/Steer/VoltageVolts", steerVoltage);
            Logger.recordOutput(modulePrefix + "/Steer/SupplyCurrentAmps", steerSupplyCurrent);
            Logger.recordOutput(modulePrefix + "/Steer/StatorCurrentAmps", steerStatorCurrent);
            Logger.recordOutput(modulePrefix + "/Steer/PowerWatts", steerPowerWatts);
            Logger.recordOutput(modulePrefix + "/Steer/EnergyJoules", steerMotorEnergy[i]);
            Logger.recordOutput(modulePrefix + "/Steer/TemperatureCelsius", steerTemp);

            // Accumulate totals
            totalPowerWatts += drivePowerWatts + steerPowerWatts;
        }

        // Accumulate total drivetrain energy
        if (dt > 0 && dt < 0.1)
        {
            totalDrivetrainEnergy.mut_plus(Watts.of(totalPowerWatts).times(Seconds.of(dt)));
        }

        // Log aggregate totals
        Logger.recordOutput("Swerve/Totals/PowerWatts", totalPowerWatts);
        Logger.recordOutput("Swerve/Totals/EnergyJoules", totalDrivetrainEnergy);
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request
     *            Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier)
    {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction
     *            Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction
     *            Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic()
    {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled())
        {
            DriverStation.getAlliance().ifPresent(allianceColor ->
            {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        robotState.swerve.setSpeed(Math.sqrt(Math.pow(getState().Speeds.vxMetersPerSecond, 2) + Math.pow(getState().Speeds.vxMetersPerSecond, 2)));
        robotState.swerve.setRobotPose(getState().Pose);

        // Log motor telemetry (currents, power, energy, temperature)
        logMotorTelemetry();
    }

    private void startSimThread()
    {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() ->
        {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters
     *            The pose of the robot as measured by the vision camera.
     * @param timestampSeconds
     *            The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds)
    {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));

    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters
     *            The pose of the robot as measured by the vision camera.
     * @param timestampSeconds
     *            The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs
     *            Standard deviations of the vision pose measurement
     *            in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs)
    {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
     * Get rotation.
     * Could directly use getState().Pose.getRotation() instead.
     * Used for Wheel Radius Characterization.
     */
    public Rotation2d getRotation()
    {
        return robotState.swerve.getRobotPose().getRotation();
    }

    /**
     * Get the gyro angular velocity.
     * Used for Wheel Radius Characterization.
     */
    public AngularVelocity getGyroRate()
    {
        return getPigeon2().getAngularVelocityZWorld().getValue();
    }

    /**
     * Get the angle positions of each drive motor.
     * Used for Wheel Radius Characterization.
     */
    public Angle[] getDrivePositions()
    {
        var modules = getModules();
        Angle[] values = new Angle[modules.length];
        for (int i = 0; i < values.length; i++)
        {
            values[i] = modules[i].getDriveMotor().getPosition().getValue();
        }
        return values;
    }

}
