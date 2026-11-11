/*
 * Initially from https://github.com/frc1678/C2022 and https://github.com/HuskieRobotics/3061-lib
 */

package frc.lib.factories;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.units.measure.Angle;
import frc.lib.util.CANDeviceId;
import frc.lib.util.CTREUtil;
import frc.robot.Robot;

@java.lang.SuppressWarnings(
{
        "java:S1104", "java:S116"
})

/**
 * Creates TalonFX objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the application.
 */
public class TalonFXFactory
{
    private static final double TIMEOUT_S = .1;

    /**
     * Create a Talon with our Default configuration
     */
    public static TalonFX makeDefault(CANDeviceId deviceId)
    {
        return makeTalon(deviceId, makeDefaultConfig());
    }

    /**
     * Create a Talon to follow another Talon
     */
    public static TalonFX makePermanentFollower(CANDeviceId deviceId, TalonFX leader, InvertedValue invert)
    {
        TalonFXConfiguration config = makeDefaultConfig();
        config.MotorOutput.Inverted = invert;

        final TalonFX talon = makeTalon(deviceId, config);
        talon.setControl(new StrictFollower(leader.getDeviceID()));

        // make sure the leader updates at a high frequency to keep the follower responsive
        // this happens after the leader has optimized its bus, to override any lower values
        leader.getMotorVoltage().setUpdateFrequency(200);
        leader.getTorqueCurrent().setUpdateFrequency(200);
        leader.getDutyCycle().setUpdateFrequency(200);
        return talon;
    }

    /**
     * Create a Talon with a custom configuration
     */
    public static TalonFX makeTalon(CANDeviceId deviceId, TalonFXConfiguration talonFXConfig)
    {
        TalonFX talon = new TalonFX(deviceId.getId(), deviceId.getBus());

        CTREUtil.apply(talon, talonFXConfig);

        talon.clearStickyFaults(TIMEOUT_S);

        return talon;
    }

    /**
     * Configures a Talon for our Default configuration.
     * 
     * <pre>
     *NeutralMode = Coast
     *Inverted = CounterClockwise_Positive
     *HardLimitSwitches = disabled
     *SoftwareLimitSwitches = disabled
     *DefaultRamps = 20ms
     *SupplyCurrentLimit = 60A
     *StatorCurrentLimit = 150A
     * </pre>
     * 
     * @return A TalonFXConfiguration object with the default settings.
     */
    public static TalonFXConfiguration makeDefaultConfig()
    {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.DutyCycleNeutralDeadband = 0.04;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = -1.0;
        config.MotorOutput.DutyCycleNeutralDeadband = 0.04;

        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;

        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        config.OpenLoopRamps = makeDefaultOpenLoopRampConfig();
        config.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
        config.CurrentLimits = makeDefaultCurrentLimitConfigs();

        config.Audio.BeepOnBoot = true;
        config.Audio.AllowMusicDurDisable = true;
        config.Audio.BeepOnConfig = true;

        // talon.voltageCompSaturation = 0.0; // In a set request, add .withOutput(12.0)

        return config;
    }

    /**
     * Creates a default CurrentLimitsConfigs object with reasonable defaults.
     * Supply current limit is enabled for real robots, but not for simulation.
     * Supply Current Limit = 60A, Stator Current Limit = 150A
     */
    public static final CurrentLimitsConfigs makeDefaultCurrentLimitConfigs()
    {
        return new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(Robot.isReal())
                .withSupplyCurrentLimit(Amps.of(60))
                .withSupplyCurrentLowerTime(Seconds.of(1))
                .withSupplyCurrentLowerLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(Robot.isReal())
                .withStatorCurrentLimit(Amps.of(150));
    }

    /**
     * Creates a default ClosedLoopRampsConfigs object with reasonable defaults.
     * Ramp periods = 20ms for all modes.
     */
    public static final ClosedLoopRampsConfigs makeDefaultClosedLoopRampConfig()
    {
        return new ClosedLoopRampsConfigs()
                // .withDutyCycleClosedLoopRampPeriod(Seconds.of(0.02))
                // .withTorqueClosedLoopRampPeriod(Seconds.of(0.02))
                // .withVoltageClosedLoopRampPeriod(Seconds.of(0.02));
                .withDutyCycleClosedLoopRampPeriod(Seconds.of(0.0))
                .withTorqueClosedLoopRampPeriod(Seconds.of(0.0))
                .withVoltageClosedLoopRampPeriod(Seconds.of(0.0));
    }

    /**
     * Creates a default OpenLoopRampsConfigs object with reasonable defaults.
     * Ramp periods = 20ms for all modes.
     */
    public static final OpenLoopRampsConfigs makeDefaultOpenLoopRampConfig()
    {
        return new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(Seconds.of(0))
                .withTorqueOpenLoopRampPeriod(Seconds.of(0))
                .withVoltageOpenLoopRampPeriod(Seconds.of(0));
    }

    /**
     * Configures the TalonFXConfiguration to use software limits.
     * 
     * @param config
     *            The TalonFXConfiguration to modify.
     * @param reverseMinPosition
     *            The minimum position for the reverse soft limit.
     * @param forwardMaxPosition
     *            The maximum position for the forward soft limit.
     * @return The modified TalonFXConfiguration with software limits set.
     */
    public static final TalonFXConfiguration withSoftLimits(TalonFXConfiguration config, Angle reverseMinPosition, Angle forwardMaxPosition)
    {
        config.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(forwardMaxPosition)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(reverseMinPosition);
        return config;
    }

    /**
     * Adds a CANcoder to the config, in FusedCANcoder mode. Sets up the ratios between the motor, sensor, and mechanism
     * 
     * @param config
     *            The TalonFXConfiguration to modify.
     * @param cancoder
     *            The CANcoder to add.
     * @param rotorToSensorRatio
     *            The ratio of the rotor to the sensor.
     * @param sensorToMechanismRatio
     *            The ratio of the sensor to the mechanism.
     */
    public static void addCANcoder(TalonFXConfiguration config, CANcoder cancoder, double rotorToSensorRatio, double sensorToMechanismRatio)
    {
        config.Feedback = new FeedbackConfigs()
                .withRotorToSensorRatio(rotorToSensorRatio)
                .withSensorToMechanismRatio(sensorToMechanismRatio)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withFeedbackRemoteSensorID(cancoder.getDeviceID());
    }
}