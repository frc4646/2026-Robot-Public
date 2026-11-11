package frc.lib.servomotor;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import java.util.Optional;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.factories.TalonFXFactory;
import frc.lib.factories.TalonFXSFactory;
import frc.lib.motor.StatusSignalTypes;
import frc.lib.util.CANDeviceId;
import frc.lib.util.CTREUtil;
import frc.lib.util.DistanceAngleConverter;

/**
 * Configuration for a servo motor, which can be a TalonFX or TalonFXS.
 * This class holds the configuration parameters for the servo motor, including
 * its ID, name, roller radius, follower configuration, CANCoder configuration,
 * soft limits, and more.
 */
public abstract class ServoMotorConfig
{
    public final String name;
    public final CANDeviceId id;
    public final DistanceAngleConverter converter;

    public StatusSignalTypes signalTypes = StatusSignalTypes.All;

    // TODO these should be arrays to allow for more than 1 follower
    public Optional<CANDeviceId> followerId = Optional.empty();
    public InvertedValue followerInvert = InvertedValue.Clockwise_Positive;

    /**
     * CANcoder ID for the servo motor. If this is set, the servo motor will use the CANCoder as the sensor.
     * If not set, it will use the motor's built-in sensor.
     */
    public Optional<CANDeviceId> cancoderId = Optional.empty();
    public Optional<CANcoderConfiguration> cancoderConfig = Optional.empty();

    /**
     * Minimum position for the negative direction soft limit and bounds limiting.
     */
    public Angle kMinPosition = Degrees.of(Double.NEGATIVE_INFINITY);
    /**
     * Maximum position for the positive direction soft limit and bounds limiting.
     */
    public Angle kMaxPosition = Degrees.of(Double.POSITIVE_INFINITY);

    public double highTripCurrent = 30;
    public double lowTripCurrent = 20;
    public int highStabilityCycleCount = 6;
    public int lowStabilityCycleCount = 15;

    // Moment of Inertia for sim
    public MomentOfInertia momentOfInertia = KilogramSquareMeters.of(0.5);

    protected ServoMotorConfig(String name, CANDeviceId id, Distance rollerRadius)
    {
        this.name = name;
        this.id = id;
        this.converter = new DistanceAngleConverter(rollerRadius);
    }

    public ServoMotorConfig withSignalTypes(StatusSignalTypes signalTypes)
    {
        this.signalTypes = signalTypes;
        return this;
    }

    /**
     * Change the TalonFX Config to use the CANCoder as the sensor.
     * CANcoder needs to be fully configured prior to this or {@link CTREUtil}.apply needs to be called on it
     * 
     * @param cancoderId
     * @param cancoderConfig
     * @param rotorToSensorRatio
     *            Gearbox Ratio from the motor to the CANcoder
     * @param sensorToMechanismRatio
     *            Ratio from CANcoder to the actual mechanism. Usually 1.0 since the CANcoder should be measuring the output
     *            shaft's angle
     * @return
     */
    public abstract ServoMotorConfig withCANcoder(CANDeviceId cancoderId, CANcoderConfiguration cancoderConfig, double rotorToSensorRatio,
            double sensorToMechanismRatio);

    public abstract ServoMotorConfig withFollower(CANDeviceId idFollower, InvertedValue followerInvert);

    /**
     * Change just the sensor units based on the gear ratio. Output / Input gears
     * 
     * @param sensorToMechanismRatio
     *            Ratio from sensor to the actual mechanism. Usually 1.0 if a CANcoder, otherwise is the gearbox's gear
     *            ratio
     * @return
     */
    public abstract ServoMotorConfig withGearRatio(double sensorToMechanismRatio);

    /**
     * Change the Config to apply soft limits
     * 
     * @param reverseMinPosition
     *            minimum position
     * @param forwardMaxPosition
     *            maximum position
     * @return
     */
    public abstract ServoMotorConfig withSoftLimits(Angle reverseMinPosition, Angle forwardMaxPosition);

    /**
     * Change the Config to apply soft limits
     * 
     * @param reverseMinPosition
     *            minimum position
     * @param forwardMaxPosition
     *            maximum position
     * @return
     */
    public abstract ServoMotorConfig withSoftLimits(Distance reverseMinPosition, Distance forwardMaxPosition);

    /**
     * Configuration for a servo motor using a TalonFX (Krakens or Falcons)
     */
    public static class ServoMotorConfigFX extends ServoMotorConfig
    {
        public final TalonFXConfiguration motorConfig;

        /**
         * Create a ServoMotorConfigFX with a name, ID.
         * 
         * @param name
         *            name of the servo motor
         * @param id
         *            CAN ID of the servo motor
         */
        public ServoMotorConfigFX(String name, CANDeviceId id)
        {
            this(name, id, Inches.one());
        }

        /**
         * Create a ServoMotorConfigFX with a name, ID, and roller radius.
         * 
         * @param name
         *            name of the servo motor
         * @param id
         *            CAN ID of the servo motor
         * @param rollerRadius
         *            radius of the roller in inches
         */
        public ServoMotorConfigFX(String name, CANDeviceId id, Distance rollerRadius)
        {
            super(name, id, rollerRadius);
            motorConfig = TalonFXFactory.makeDefaultConfig();

            // why is this different than the default from TalonFXFactory?
            motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        @Override
        public ServoMotorConfigFX withCANcoder(CANDeviceId cancoderId, CANcoderConfiguration cancoderConfig, double rotorToSensorRatio,
                double sensorToMechanismRatio)
        {
            this.cancoderId = Optional.ofNullable(cancoderId);
            this.cancoderConfig = Optional.ofNullable(cancoderConfig);

            motorConfig.Feedback.withRotorToSensorRatio(rotorToSensorRatio);
            motorConfig.Feedback.withSensorToMechanismRatio(sensorToMechanismRatio);
            motorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);
            motorConfig.Feedback.withFeedbackRemoteSensorID(cancoderId.getId());

            return this;
        }

        @Override
        public ServoMotorConfigFX withGearRatio(double sensorToMechanismRatio)
        {
            this.motorConfig.Feedback.withSensorToMechanismRatio(sensorToMechanismRatio);

            return this;
        }

        @Override
        public ServoMotorConfigFX withSoftLimits(Angle reverseMinPosition, Angle forwardMaxPosition)
        {
            kMaxPosition = forwardMaxPosition;
            kMinPosition = reverseMinPosition;

            motorConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
            motorConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(forwardMaxPosition);
            motorConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
            motorConfig.SoftwareLimitSwitch.withReverseSoftLimitThreshold(reverseMinPosition);

            return this;
        }

        @Override
        public ServoMotorConfigFX withSoftLimits(Distance reverseMinPosition, Distance forwardMaxPosition)
        {
            return withSoftLimits(
                    converter.toAngle(reverseMinPosition),
                    converter.toAngle(forwardMaxPosition));
        }

        @Override
        public ServoMotorConfigFX withFollower(CANDeviceId idFollower, InvertedValue followerInvert)
        {
            this.followerId = Optional.ofNullable(idFollower);
            this.followerInvert = followerInvert;

            return this;
        }
    }

    /**
     * Configuration for a servo motor using a TalonFXS (Minions or NEOs)
     */
    public static class ServoMotorConfigFXS extends ServoMotorConfig
    {
        public TalonFXSConfiguration motorConfig;

        public ServoMotorConfigFXS(String name, CANDeviceId id, Distance rollerRadius, MotorArrangementValue motorType)
        {
            super(name, id, rollerRadius);
            motorConfig = TalonFXSFactory.makeDefaultConfig(motorType);

            // why is this different than the default from TalonFXSFactory?
            motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        @Override
        public ServoMotorConfigFXS withCANcoder(CANDeviceId cancoderId, CANcoderConfiguration cancoderConfig, double rotorToSensorRatio,
                double sensorToMechanismRatio)
        {
            this.cancoderId = Optional.ofNullable(cancoderId);
            this.cancoderConfig = Optional.ofNullable(cancoderConfig);

            motorConfig.ExternalFeedback.withRotorToSensorRatio(rotorToSensorRatio);
            motorConfig.ExternalFeedback.withSensorToMechanismRatio(sensorToMechanismRatio);
            motorConfig.ExternalFeedback.withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.FusedCANcoder);
            motorConfig.ExternalFeedback.withFeedbackRemoteSensorID(cancoderId.getId());

            return this;
        }

        @Override
        public ServoMotorConfigFXS withGearRatio(double sensorToMechanismRatio)
        {
            this.motorConfig.ExternalFeedback.withSensorToMechanismRatio(sensorToMechanismRatio);

            return this;
        }

        @Override
        public ServoMotorConfigFXS withSoftLimits(Angle reverseMinPosition, Angle forwardMaxPosition)
        {
            kMaxPosition = forwardMaxPosition;
            kMinPosition = reverseMinPosition;

            motorConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
            motorConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(forwardMaxPosition);
            motorConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
            motorConfig.SoftwareLimitSwitch.withReverseSoftLimitThreshold(reverseMinPosition);

            return this;
        }

        @Override
        public ServoMotorConfigFXS withSoftLimits(Distance reverseMinPosition, Distance forwardMaxPosition)
        {
            return withSoftLimits(
                    converter.toAngle(reverseMinPosition),
                    converter.toAngle(forwardMaxPosition));
        }

        @Override
        public ServoMotorConfigFXS withFollower(CANDeviceId idFollower, InvertedValue followerInvert)
        {
            this.followerId = Optional.ofNullable(idFollower);
            this.followerInvert = followerInvert;

            return this;
        }
    }
}
