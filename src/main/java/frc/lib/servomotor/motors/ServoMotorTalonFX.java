package frc.lib.servomotor.motors;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DifferentialFollower;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.factories.CANcoderFactory;
import frc.lib.factories.ControlRequestFactory;
import frc.lib.factories.TalonFXFactory;
import frc.lib.motor.MotorActualsAutoLogged;
import frc.lib.motor.MotorTargets.TargetModes;
import frc.lib.motor.TalonStatusSignals;
import frc.lib.servomotor.ServoMotorConfig.ServoMotorConfigFX;
import frc.lib.util.CTREUtil;

public class ServoMotorTalonFX extends ServoMotor
{

    // protected ServoMotorConfig config;
    protected TalonFX talon;
    protected Optional<CANcoder> cancoder = Optional.empty();
    protected Optional<TalonFX> talonFollower = Optional.empty();

    private final TalonStatusSignals statusSignals;
    private Optional<TalonStatusSignals> followerStatusSignals = Optional.empty();

    public ServoMotorTalonFX(ServoMotorConfigFX config)
    {
        super(config);

        talon = TalonFXFactory.makeTalon(config.id, config.motorConfig);

        // get the status signals
        statusSignals = new TalonStatusSignals(talon, config.signalTypes);

        // optimize status updates
        optimizeBus(talon, statusSignals);

        // if the follower was configured, create it
        if (config.followerId.isPresent())
        {
            TalonFX follower = TalonFXFactory.makePermanentFollower(config.followerId.get(), talon, config.followerInvert);
            talonFollower = Optional.of(follower);

            // Create status signals for follower (currents only for minimal CAN overhead)
            TalonStatusSignals followerSignals = TalonStatusSignals.forFollower(follower);
            followerStatusSignals = Optional.of(followerSignals);
            followerActuals = Optional.of(new MotorActualsAutoLogged());

            optimizeBus(follower, followerSignals);
        }

        // if the cancoder was configured, create it
        if (config.cancoderConfig.isPresent() && config.cancoderId.isPresent())
        {
            cancoder = Optional.of(CANcoderFactory.make(config.cancoderId.get(), config.cancoderConfig.get()));
        }
    }

    @Override
    public void readInputs()
    {
        // Batch refresh all signals (leader + follower if present)
        if (followerStatusSignals.isPresent())
        {
            BaseStatusSignal.refreshAll(
                    concatenateSignals(statusSignals.get(), followerStatusSignals.get().get()));
        }
        else
        {
            BaseStatusSignal.refreshAll(statusSignals.get());
        }

        // Update and log leader actuals
        statusSignals.updateSensorsFromSignals(actuals, getConverter());
        Logger.processInputs(LoggerPrefix + getName() + LoggerPostfixActuals, getActuals());

        // Update and log follower actuals if present
        if (followerStatusSignals.isPresent() && followerActuals.isPresent())
        {
            followerStatusSignals.get().updateSensorsFromSignals(followerActuals.get(), getConverter());
            Logger.processInputs(LoggerPrefix + getName() + LoggerPostfixFollowerActuals, followerActuals.get());
        }

        highCurrentTrip.calculate();
        lowCurrentTrip.calculate();
    }

    private BaseStatusSignal[] concatenateSignals(BaseStatusSignal[] a, BaseStatusSignal[] b)
    {
        BaseStatusSignal[] result = new BaseStatusSignal[a.length + b.length];
        System.arraycopy(a, 0, result, 0, a.length);
        System.arraycopy(b, 0, result, a.length, b.length);
        return result;
    }

    @Override
    public void setTargetDutyCycle(double dutyCycle)
    {
        talon.setControl(ControlRequestFactory.withDutyCycle(dutyCycle));
        setpoints.mode = TargetModes.DutyCycle;
        setpoints.dutyCycle = dutyCycle;

        logTargets();
    }

    @Override
    public void setTargetAngle(Angle position)
    {
        talon.setControl(ControlRequestFactory.withPosition(position));
        setpoints.mode = TargetModes.Position;
        setpoints.positionRotations = position.in(Rotations);
        setpoints.positionInches = getConverter().toDistance(position).in(Inches);

        logTargets();
    }

    @Override
    public void setTargetAngle(Angle position, Voltage feedForaward)
    {
        talon.setControl(ControlRequestFactory.withPosition(position).withFeedForward(feedForaward));
        setpoints.mode = TargetModes.Position;
        setpoints.positionRotations = position.in(Rotations);
        setpoints.positionInches = getConverter().toDistance(position).in(Inches);

        logTargets();
    }

    @Override
    public void setTargetAngle(Angle position, int slotId)
    {
        talon.setControl(ControlRequestFactory.withPosition(position).withSlot(slotId));
        setpoints.mode = TargetModes.Position;
        setpoints.positionRotations = position.in(Rotations);
        setpoints.positionInches = getConverter().toDistance(position).in(Inches);

        logTargets();
    }

    @Override
    public void setTargetAngle(Angle position, AngularVelocity velocity, AngularAcceleration acceleration)
    {
        talon.setControl(ControlRequestFactory.withDynamicPosition(clampPosition(position), velocity, acceleration));
        setpoints.mode = TargetModes.Position;
        setpoints.positionRotations = position.in(Rotations);
        setpoints.positionInches = getConverter().toDistance(position).in(Inches);

        logTargets();
    }

    @Override
    public void setTargetAngle(Angle position, AngularVelocity velocity, AngularAcceleration acceleration, int slotId)
    {
        talon.setControl(ControlRequestFactory.withDynamicPosition(clampPosition(position), velocity, acceleration).withSlot(slotId));
        setpoints.mode = TargetModes.Position;
        setpoints.positionRotations = position.in(Rotations);
        setpoints.positionInches = getConverter().toDistance(position).in(Inches);

        logTargets();
    }

    @Override
    public void setTargetVelocity(AngularVelocity velocity)
    {
        talon.setControl(ControlRequestFactory.withVelocity(velocity));
        setpoints.mode = TargetModes.Velocity;
        setpoints.velocityRotationsPerSecond = velocity.in(RotationsPerSecond);
        setpoints.velocityInchesPerSecond = getConverter().toLinearVelocity(velocity).in(InchesPerSecond);

        logTargets();
    }

    @Override
    public void setTargetVelocity(AngularVelocity velocity, int slotId)
    {
        talon.setControl(ControlRequestFactory.withVelocity(velocity).withSlot(slotId));
        setpoints.mode = TargetModes.Velocity;
        setpoints.velocityRotationsPerSecond = velocity.in(RotationsPerSecond);
        setpoints.velocityInchesPerSecond = getConverter().toLinearVelocity(velocity).in(InchesPerSecond);

        logTargets();
    }

    @Override
    public void setTargetVoltage(Voltage voltage)
    {
        talon.setControl(ControlRequestFactory.withVoltage(voltage));
        setpoints.mode = TargetModes.Voltage;
        setpoints.voltage = voltage.in(Volts);

        logTargets();
    }

    @Override
    public void setTargetNeutral()
    {
        talon.setControl(ControlRequestFactory.withNeutral());
    }

    @Override
    public void setTargetCoast()
    {
        talon.setControl(ControlRequestFactory.withCoast());
    }

    @Override
    public void setConfigCurrentPosition(Angle position)
    {
        CTREUtil.tryUntilOK(() -> talon.setPosition(position), talon.getDeviceID());

    }

    @Override
    public void setConfigNeutralMode(NeutralModeValue mode)
    {
        CTREUtil.tryUntilOK(() -> talon.setNeutralMode(mode), talon.getDeviceID());
    }

    @Override
    public void setConfigSoftLimitsEnable(boolean fwd, boolean rev)
    {

        var config = new SoftwareLimitSwitchConfigs();
        CTREUtil.refreshConfiguration(talon, config);
        config.ForwardSoftLimitEnable = fwd;
        config.ReverseSoftLimitEnable = rev;
        CTREUtil.apply(talon, config);
    }

    @Override
    public void setConfigCurrentLimitsEnable(boolean statorLimit, boolean supplyLimit)
    {
        var config = new CurrentLimitsConfigs();
        CTREUtil.refreshConfiguration(talon, config);
        config.StatorCurrentLimitEnable = statorLimit;
        config.SupplyCurrentLimitEnable = supplyLimit;
        CTREUtil.apply(talon, config);
    }

    @Override
    public SoftwareLimitSwitchConfigs getSoftwareLimitSwitchConfigs()
    {
        var config = new SoftwareLimitSwitchConfigs();
        CTREUtil.refreshConfiguration(talon, config);
        return config;
    }

    @Override
    public CurrentLimitsConfigs getCurrentLimitsConfigs()
    {
        var config = new CurrentLimitsConfigs();
        CTREUtil.refreshConfiguration(talon, config);
        return config;
    }

    public void setAsDifferentialFollower(ServoMotorTalonFX leader)
    {
        if (talonFollower.isPresent())
        {
            talon.setControl(new DifferentialFollower(0, MotorAlignmentValue.Opposed));
        }
    }

    public void setCustomRequest(ControlRequest controlRequest)
    {
        talon.setControl(controlRequest);

        if (controlRequest instanceof MotionMagicVoltage)
        {
            MotionMagicVoltage motionMagicRequest = (MotionMagicVoltage) controlRequest;
            setpoints.mode = TargetModes.Position;
            var position = motionMagicRequest.getPositionMeasure();
            setpoints.positionRotations = position.in(Rotations);
            setpoints.positionInches = getConverter().toDistance(position).in(Inches);
        }
        else if (controlRequest instanceof VoltageOut)
        {
            VoltageOut voltageRequest = (VoltageOut) controlRequest;
            setpoints.mode = TargetModes.Voltage;
            setpoints.voltage = voltageRequest.getOutputMeasure().in(Volts);
        }
        else if (controlRequest instanceof DutyCycleOut)
        {
            DutyCycleOut dutyCycleRequest = (DutyCycleOut) controlRequest;
            setpoints.mode = TargetModes.DutyCycle;
            setpoints.dutyCycle = dutyCycleRequest.Output;
        }
        else if (controlRequest instanceof VelocityTorqueCurrentFOC)
        {
            VelocityTorqueCurrentFOC velocityRequest = (VelocityTorqueCurrentFOC) controlRequest;
            setpoints.mode = TargetModes.Velocity;
            var velocity = velocityRequest.getVelocityMeasure();
            setpoints.velocityRotationsPerSecond = velocity.in(RotationsPerSecond);
            setpoints.velocityInchesPerSecond = getConverter().toLinearVelocity(velocity).in(InchesPerSecond);
        }
        else if (controlRequest instanceof DynamicMotionMagicVoltage)
        {
            DynamicMotionMagicVoltage dynamicMotionMagicRequest = (DynamicMotionMagicVoltage) controlRequest;
            setpoints.mode = TargetModes.Position;
            var position = dynamicMotionMagicRequest.getPositionMeasure();
            setpoints.positionRotations = position.in(Rotations);
            setpoints.positionInches = getConverter().toDistance(position).in(Inches);
        }
    }

    /**
     * Optimizes the bus utilization for the TalonFX motor controller.
     * This method sets the update frequency for status signals and optimizes bus utilization.
     *
     * @param talon
     *            the TalonFX motor controller to optimize
     * @param signals
     *            the Signals object containing status signals
     */
    private void optimizeBus(TalonFX talon, TalonStatusSignals signals)
    {
        // optimize status updates
        if (talon.getNetwork().getName().contains("rio"))
        {
            CTREUtil.tryUntilOK(() -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals.get()), talon.getDeviceID());
        }
        else
        {
            CTREUtil.tryUntilOK(() -> BaseStatusSignal.setUpdateFrequencyForAll(100.0, signals.get()), talon.getDeviceID());
        }
        CTREUtil.tryUntilOK(() -> talon.optimizeBusUtilization(), talon.getDeviceID());
    }

    public TalonFX getTalon()
    {
        return talon;
    }
}
