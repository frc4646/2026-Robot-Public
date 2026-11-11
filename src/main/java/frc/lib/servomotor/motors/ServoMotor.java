package frc.lib.servomotor.motors;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.motor.MotorActualsAutoLogged;
import frc.lib.motor.MotorTargetsAutoLogged;
import frc.lib.servomotor.ServoMotorConfig;
import frc.lib.util.DistanceAngleConverter;
import frc.lib.util.StabilityCounter;

public abstract class ServoMotor
{
    protected static final String LoggerPrefix = "RealOutputs/";
    protected static final String LoggerPostfixActuals = "/Actuals";
    protected static final String LoggerPostfixTargets = "/Targets";
    protected static final String LoggerPostfixFollowerActuals = "/Follower/Actuals";

    protected final ServoMotorConfig config;
    protected final MotorActualsAutoLogged actuals;
    protected final MotorTargetsAutoLogged setpoints;
    protected Optional<MotorActualsAutoLogged> followerActuals = Optional.empty();

    protected final StabilityCounter highCurrentTrip;
    protected final StabilityCounter lowCurrentTrip;

    public ServoMotor(ServoMotorConfig config)
    {
        this.config = config;
        actuals = new MotorActualsAutoLogged();
        setpoints = new MotorTargetsAutoLogged();

        // setup the stabilitiy counters
        highCurrentTrip = new StabilityCounter(config.highStabilityCycleCount, () -> getActuals().currentStatorAmps > config.highTripCurrent);
        lowCurrentTrip = new StabilityCounter(config.lowStabilityCycleCount, () -> getActuals().currentStatorAmps < config.lowTripCurrent);
    }

    public MotorTargetsAutoLogged getTargets()
    {
        return setpoints;
    }

    public MotorActualsAutoLogged getActuals()
    {
        return actuals;
    }

    /**
     * Get the follower motor actuals, if a follower is present.
     *
     * @return Optional containing follower actuals, or empty if no follower
     */
    public Optional<MotorActualsAutoLogged> getFollowerActuals()
    {
        return followerActuals;
    }

    /**
     * Check if this motor has a follower configured.
     *
     * @return true if a follower motor is present
     */
    public boolean hasFollower()
    {
        return followerActuals.isPresent();
    }

    public String getName()
    {
        return config.name;
    }

    public DistanceAngleConverter getConverter()
    {
        return config.converter;
    }

    /**
     * Read the inputs from the motor and update the {@link MotorActualsAutoLogged} object.
     * This method should be called periodically to keep the actuals updated.
     *
     * @param actuals
     */
    public abstract void readInputs();

    public void logTargets()
    {
        Logger.processInputs(LoggerPrefix + getName() + LoggerPostfixTargets, getTargets());
    }

    /**
     * Set the duty cycle for the motor.
     * 
     * @param dutyCycle
     */
    public abstract void setTargetDutyCycle(double dutyCycle);

    /**
     * Set the target position for the motor.
     * Uses Motion Magic. Uses the last PID Slot ID that was used.
     * 
     * @param position
     */
    public abstract void setTargetAngle(Angle position);

    /**
     * Set the target position for the motor.
     * Uses Motion Magic. Uses the last PID Slot ID that was used.
     * Includes feed forward voltage
     * 
     * @param position
     */
    public abstract void setTargetAngle(Angle position, Voltage feedForwardVoltage);

    /**
     * Set the target position for the motor. Uses Motion Magic.
     * 
     * @param position
     * @param slotId
     */
    public abstract void setTargetAngle(Angle position, int slotId);

    /**
     * Set the target position for the motor with dynamic velocity and acceleration.
     * Uses Motion Magic. Uses the last PID Slot ID that was used.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     */
    public abstract void setTargetAngle(Angle position, AngularVelocity velocity, AngularAcceleration acceleration);

    /**
     * Set the target position for the motor with dynamic velocity and acceleration.
     * Uses Motion Magic.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     * @param slotId
     */
    public abstract void setTargetAngle(Angle position, AngularVelocity velocity, AngularAcceleration acceleration, int slotId);

    /**
     * Set the target velocity for the motor.
     * Uses Velocity Voltage Control. Uses the last PID Slot ID that was used.
     * 
     * @param velocity
     */
    public abstract void setTargetVelocity(AngularVelocity velocity);

    /**
     * Set the target velocity for the motor. Uses Velocity Voltage Control.
     * 
     * @param velocity
     * @param slotId
     */
    public abstract void setTargetVelocity(AngularVelocity velocity, int slotId);

    /**
     * Set the voltage output for the motor.
     * 
     * @param voltage
     */
    public abstract void setTargetVoltage(Voltage voltage);

    public abstract void setTargetNeutral();

    public abstract void setTargetCoast();

    /**
     * Set the current position of the motor as zero. Used to set what 0 means for position control.
     * 
     * @return StatusCode indicating success or failure
     */
    public void setConfigCurrentPositionAsZero()
    {
        setConfigCurrentPosition(Degrees.of(0));
    }

    /**
     * Set the current position of the motor to any angle. Used to set what 0 means for position control.
     * 
     * @param position
     * @return StatusCode indicating success or failure
     */
    public abstract void setConfigCurrentPosition(Angle position);

    /**
     * Set the neutral mode for the motor. This is a configuration change, and is a blocking call.
     * 
     * @param mode
     */
    public abstract void setConfigNeutralMode(NeutralModeValue mode);

    /**
     * Set the soft limits for the motor. This is a configuration change, and is a blocking call.
     * 
     * @param fwd
     * @param rev
     */
    public abstract void setConfigSoftLimitsEnable(boolean fwd, boolean rev);

    /**
     * Set the current limits for the motor. This is a configuration change, and is a blocking call.
     * 
     * @param statorLimit
     * @param supplyLimit
     */
    public abstract void setConfigCurrentLimitsEnable(boolean statorLimit, boolean supplyLimit);

    /**
     * Get the software limit switch configurations for the motor.
     *
     * @return SoftwareLimitSwitchConfigs
     */
    public abstract SoftwareLimitSwitchConfigs getSoftwareLimitSwitchConfigs();

    /**
     * Get the current limits configurations for the motor.
     *
     * @return CurrentLimitsConfigs
     */
    public abstract CurrentLimitsConfigs getCurrentLimitsConfigs();

    public Trigger getHighCurrentTripTrigger()
    {
        return highCurrentTrip.asTrigger();
    }

    public Trigger getLowCurrentTripTrigger()
    {
        return lowCurrentTrip.asTrigger();
    }

    public void setTargetDistance(Distance position)
    {
        setTargetAngle(getConverter().toAngle(position));
    }

    /**
     * Set the target position for the motor. Uses Motion Magic.
     * 
     * @param position
     * @param slotId
     */
    public void setTargetDistance(Distance position, int slotId)
    {
        setTargetAngle(getConverter().toAngle(position), slotId);
    }

    /**
     * Set the target position for the motor with dynamic velocity and acceleration.
     * Uses Motion Magic. Uses the last PID Slot ID that was used.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     */
    public void setTargetDistance(Distance position, LinearVelocity velocity, LinearAcceleration acceleration)
    {
        setTargetAngle(getConverter().toAngle(position), getConverter().toAngularVelocity(velocity), getConverter().toAngularAcceleration(acceleration));
    }

    /**
     * Set the target position for the motor with dynamic velocity and acceleration.
     * Uses Motion Magic.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     * @param slotId
     */
    public void setTargetDistance(Distance position, LinearVelocity velocity, LinearAcceleration acceleration, int slotId)
    {
        setTargetAngle(getConverter().toAngle(position), getConverter().toAngularVelocity(velocity), getConverter().toAngularAcceleration(acceleration),
                slotId);
    }

    /**
     * Set the target velocity for the motor.
     * Uses Velocity Voltage Control. Uses the last PID Slot ID that was used.
     * 
     * @param velocity
     */
    public void setTargetLinearVelocity(LinearVelocity velocity)
    {
        setTargetVelocity(getConverter().toAngularVelocity(velocity));
    }

    /**
     * Set the target velocity for the motor. Uses Velocity Voltage Control.
     * 
     * @param velocity
     * @param slotId
     */
    public void setTargetLinearVelocity(LinearVelocity velocity, int slotId)
    {
        setTargetVelocity(getConverter().toAngularVelocity(velocity), slotId);
    }

    public abstract void setCustomRequest(ControlRequest controlRequest);

    /**
     * Clamp the input position between Min and Max positions in the config
     */
    protected Angle clampPosition(Angle position)
    {
        return Rotations.of(
                MathUtil.clamp(position.in(Rotations),
                        config.kMinPosition.in(Rotations),
                        config.kMaxPosition.in(Rotations)));
    }
}
