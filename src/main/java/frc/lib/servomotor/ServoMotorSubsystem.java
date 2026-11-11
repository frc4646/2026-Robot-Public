package frc.lib.servomotor;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Joules;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutEnergy;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.motor.MotorActualsAutoLogged;
import frc.lib.motor.MotorTargetsAutoLogged;
import frc.lib.servomotor.motors.ServoMotor;
import frc.lib.util.Util;

/**
 * Extend this subsystem to create a new controllable ServoMotor
 */
public class ServoMotorSubsystem<ServoMotorType extends ServoMotor> extends SubsystemBase
{
    private final ServoMotorType servoMotor;
    private final ServoMotorCommands commands = new ServoMotorCommands();
    private final ServoMotorTriggers triggers = new ServoMotorTriggers();

    private double lastTimestamp = 0.0;
    private MutEnergy totalEnergy = Joules.zero().mutableCopy();

    /**
     * Create a new ServoMotorSubsystem
     * 
     * @param servoMotor
     *            The servo motor to control. This should be a subclass of {@link ServoMotor}, like {@link ServoMotorFX} or
     *            {@link ServoMotorFXS}.
     */
    public ServoMotorSubsystem(ServoMotorType servoMotor)
    {
        super(servoMotor.getName());
        this.servoMotor = servoMotor;
        setDefaultCommand(commands.setTargetDutyCycle(() -> 0.0).withName(getName() + " Default 0%"));
    }

    @Override
    public void periodic()
    {
        double timestamp = Timer.getFPGATimestamp();
        servoMotor.readInputs();

        // Energy tracking: Power = Voltage * Current, Energy = Power * Time
        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;

        // Calculate instantaneous power (leader)
        Power leaderPower = getActuals().getAppliedVoltage().times(getActuals().getSupplyCurrent());
        Power totalPower = leaderPower;

        // Add follower power if present
        if (servoMotor.hasFollower())
        {
            MotorActualsAutoLogged follower = servoMotor.getFollowerActuals().get();
            Power followerPower = follower.getAppliedVoltage().times(follower.getSupplyCurrent());
            totalPower = leaderPower.plus(followerPower);
            Logger.recordOutput(getName() + "/Follower/Actuals/PowerWatts", followerPower);
        }

        // Accumulate energy (joules) - only accumulate after first loop
        if (dt > 0 && dt < 0.1) // Guard against large dt on startup or after pause
        {
            totalEnergy.mut_plus(totalPower.times(Seconds.of(dt)));
        }

        // Log combined power and energy metrics
        Logger.recordOutput(getName() + "/Actuals/PowerWatts", totalPower);
        Logger.recordOutput(getName() + "/Actuals/EnergyJoules", totalEnergy);

        Logger.recordOutput(getName() + "/PeriodicMS", (Timer.getFPGATimestamp() - timestamp) * 1000.0);
    }

    /**
     * Get the underlying servo motor object
     * 
     * @return The servo motor
     */
    public ServoMotorType getServoMotor()
    {
        return servoMotor;
    }

    /**
     * Get the actual signals of the motor
     * 
     * @return MotorActuals
     */
    public MotorActualsAutoLogged getActuals()
    {
        return servoMotor.getActuals();
    }

    /**
     * Get the target values of the motor
     * 
     * @return MotorTargets
     */
    public MotorTargetsAutoLogged getTargets()
    {
        return servoMotor.getTargets();
    }

    /**
     * Get the error between target and actual angle
     * 
     * @return Target Angle - Actual Angle
     */
    public Angle getErrorAngle()
    {
        return getTargets().getAngle().minus(getActuals().getAngle());
    }

    /**
     * Get the error between target and actual distance
     * 
     * @return Target Distance - Actual Distance
     */
    public Distance getErrorDistance()
    {
        return getTargets().getDistance().minus(getActuals().getDistance());
    }

    /**
     * Get the error between target and actual angular velocity
     * 
     * @return Target Angular Velocity - Actual Angular Velocity
     */
    public AngularVelocity getErrorAngularVelocity()
    {
        return getTargets().getAngularVelocity().minus(getActuals().getAngularVelocity());
    }

    /**
     * Get the error between target and actual linear velocity
     * 
     * @return Target Linear Velocity - Actual Linear Velocity
     */
    public LinearVelocity getErrorLinearVelocity()
    {
        return getTargets().getLinearVelocity().minus(getActuals().getLinearVelocity());
    }

    /**
     * Get the generic commands for controlling a servo motor
     * 
     * @return ServoMotorCommands
     */
    public ServoMotorCommands getServoCommands()
    {
        return commands;
    }

    /**
     * Set of generic commands for controlling a servo motor. Building blocks that can be added onto within an
     * implementation of a ServoMotorSubsystem
     */
    public class ServoMotorCommands
    {
        /**
         * Create a duty cycle command. Continuously updates the target while running. Sets to 0 when ends
         * 
         * @param dutyCycle
         *            Target duty cycle, -1.0 to 1.0
         * @return {@link RunCommand} with 0% on end
         */
        public Command setTargetDutyCycle(DoubleSupplier dutyCycle)
        {
            return runEnd(() ->
            {
                servoMotor.setTargetDutyCycle(dutyCycle.getAsDouble());
            }, () ->
            {
                servoMotor.setTargetDutyCycle(0.0);
            }).withName(getName() + " DutyCycleControl");
        }

        /**
         * Create a duty cycle command. Sets to 0 when ends
         * 
         * @param dutyCycle
         *            Target duty cycle, -1.0 to 1.0
         * @return {@link RunCommand} with 0% on end
         */
        public Command setTargetDutyCycle(double dutyCycle)
        {
            return setTargetDutyCycle(() -> dutyCycle);
        }

        /**
         * Create a duty cycle command that ends when you're above the High trip current
         * 
         * @param dutyCycle
         *            Target duty cycle, -1.0 to 1.0
         * @return {@link Command} that ends when above the High current trip
         */
        public Command setTargetDutyCycleHighCurrentTripCommmand(DoubleSupplier dutyCycle)
        {
            return setTargetDutyCycle(dutyCycle)
                    .withDeadline(new WaitCommand(.1)
                            .andThen(new WaitUntilCommand(servoMotor.getHighCurrentTripTrigger())))
                    .withName(getName() + " DutyCycleHighCurrentTrip");
        }

        /**
         * Create a duty cycle command that ends when you're below the Low trip current
         * 
         * @param dutyCycle
         *            Target duty cycle, -1.0 to 1.0
         * @return {@link Command} that ends when below the Low trip current
         */
        public Command setTargetDutyCycleLowCurrentTripCommmand(DoubleSupplier dutyCycle)
        {
            return setTargetDutyCycle(dutyCycle)
                    .withDeadline(new WaitCommand(.1)
                            .andThen(new WaitUntilCommand(servoMotor.getLowCurrentTripTrigger())))
                    .withName(getName() + " DutyCycleLowCurrentTrip");
        }

        /**
         * Create an angular position command. Continuously updates the target angle while running
         * 
         * @param angleSupplier
         *            A supplier that provides the target angle
         * @return {@link RunCommand}
         */
        public Command setTargetAngle(Supplier<Angle> angleSupplier)
        {
            return run(() ->
            {
                servoMotor.setTargetAngle(angleSupplier.get());
            }).withName(getName() + " targetPositionCommand");
        }

        /**
         * Create an angular position command. Continuously updates the target angle while running
         * 
         * @param angleSupplier
         *            A supplier that provides the target angle
         * @param SlotID
         *            specify the PID Slot
         * @return {@link RunCommand}
         */
        public Command setTargetAngle(Supplier<Angle> angleSupplier, int SlotID)
        {
            return run(() ->
            {
                servoMotor.setTargetAngle(angleSupplier.get(), SlotID);
            }).withName(getName() + " targetPositionCommand");
        }

        /**
         * Create an angular position command. Continuously updates the target angle while running
         * 
         * @param angleSupplier
         *            A supplier that provides the target angle
         * @return {@link RunCommand}
         */
        public Command setTargetAngle(Supplier<Angle> angleSupplier, Supplier<Voltage> feedForwardSupplier)
        {
            return run(() ->
            {
                servoMotor.setTargetAngle(angleSupplier.get(), feedForwardSupplier.get());
            }).withName(getName() + " targetPositionCommand");
        }

        /**
         * Create an angular position command.
         * 
         * @param angle
         *            The target angle
         * @return {@link RunCommand}
         */
        public Command setTargetAngle(Angle angle)
        {
            return setTargetAngle(() -> angle);
        }

        /**
         * Create an angular position command that ends when the position is within a tolerance
         * 
         * @param angleSupplier
         *            A supplier that provides the target angle
         * @param tolerance
         *            The tolerance to consider "at target"
         * @return {@link RunCommand} with a {@link WaitUntilCommand}
         */
        public Command setTargetAngle_UntilAtAngle(Supplier<Angle> angleSupplier, Angle tolerance)
        {
            return setTargetAngle(angleSupplier)
                    .until(triggers.atTargetPosition(tolerance));
        }

        /**
         * Create an angular position command that ends when the position is within a tolerance
         * 
         * @param angle
         *            The target angle
         * @param tolerance
         *            The tolerance to consider "at target"
         * @return {@link RunCommand} with a {@link WaitUntilCommand}
         */
        public Command setTargetAngle_UntilAtAngle(Angle angle, Angle tolerance)
        {
            return setTargetAngle_UntilAtAngle(() -> angle, tolerance);
        }

        /**
         * Create a linear position command. Continuously updates the target angle while running
         * 
         * @param distanceSupplier
         *            A supplier that provides the target distance
         * @return {@link RunCommand}
         */
        public Command setTargetDistance(Supplier<Distance> distanceSupplier)
        {
            return run(() ->
            {
                servoMotor.setTargetDistance(distanceSupplier.get());
            }).withName(getName() + " targetPositionCommand");
        }

        /**
         * Create a linear position command.
         * 
         * @param distance
         *            The target distance
         * @return {@link RunCommand}
         */
        public Command setTargetDistance(Distance distance)
        {
            return setTargetDistance(() -> distance);
        }

        /**
         * Create a linear position command that ends when the position is within a tolerance
         * 
         * @param distanceSupplier
         *            A supplier that provides the target distance
         * @param tolerance
         *            The tolerance to consider "at target"
         * @return {@link RunCommand} with a {@link WaitUntilCommand}
         */
        public Command setTargetDistance_UntilAtDistance(Supplier<Distance> distanceSupplier, Distance tolerance)
        {
            return setTargetDistance(distanceSupplier)
                    .until(triggers.atTargetPosition(tolerance));
        }

        /**
         * Create a linear position command that ends when the position is within a tolerance
         * 
         * @param distance
         *            The target distance
         * @param tolerance
         *            The tolerance to consider "at target"
         * @return {@link RunCommand} with a {@link WaitUntilCommand}
         */
        public Command setTargetDistancE_UntilAtDistance(Distance distanceSupplier, Distance tolerance)
        {
            return setTargetDistance_UntilAtDistance(() -> distanceSupplier, tolerance);
        }

        /**
         * Create a linear position command with a specific PID profile slot. Continuously updates the target angle while
         * running
         * 
         * @param distanceSupplier
         *            A supplier that provides the target distance
         * @param slotID
         *            The PID profile slot to use
         * @return {@link RunCommand}
         */
        public Command setTargetDistance(Supplier<Distance> distanceSupplier, int slotID)
        {
            return run(() ->
            {
                servoMotor.setTargetDistance(distanceSupplier.get(), slotID);
            }).withName(getName() + " targetPositionCommand");
        }

        /**
         * Create a linear position command with a specific PID profile slot.
         * 
         * @param distance
         *            The target distance
         * @param slotID
         *            The PID profile slot to use
         * @return
         */
        public Command setTargetDistance(Distance distance, int slotID)
        {
            return setTargetDistance(() -> distance, slotID);
        }

        /**
         * Create a linear position command with a specific PID profile slot. Continuously updates the target angle while
         * running
         * 
         * @param distanceSupplier
         *            A supplier that provides the target distance
         * @param slotIDSupplier
         *            A supplier that provides the PID profile slot to use
         * @return {@link RunCommand}
         */
        public Command setTargetDistance(Supplier<Distance> distanceSupplier, IntSupplier slotIDSupplier)
        {
            return run(() ->
            {
                servoMotor.setTargetDistance(distanceSupplier.get(), slotIDSupplier.getAsInt());
            }).withName(getName() + " targetPositionCommand");
        }

        /**
         * Create a linear position command, with specific velocity and acceleration limits, with a specific PID profile slot.
         * Continuously updates the target angle while running
         * 
         * @param distanceSupplier
         *            A supplier that provides the target distance
         * @param mmVelocity
         *            The max velocity to use for motion magic
         * @param mmAccel
         *            The max acceleration to use for motion magic
         * @param slotId
         *            The PID profile slot to use
         * @return {@link RunCommand}
         */
        public Command setTargetDistanceDynamic(Supplier<Distance> distanceSupplier,
                LinearVelocity mmVelocity,
                LinearAcceleration mmAccel,
                int slotId)
        {
            return run(() ->
            {
                servoMotor.setTargetDistance(
                        distanceSupplier.get(),
                        mmVelocity,
                        mmAccel,
                        slotId);
            }).withName(getName() + " targetPositionCommand");
        }

        /**
         * Create a linear position command, with specific velocity and acceleration limits, with a specific PID profile slot.
         * 
         * @param distance
         *            The target distance
         * @param mmVelocity
         *            The max velocity to use for motion magic
         * @param mmAccel
         *            The max acceleration to use for motion magic
         * @param slotId
         *            The PID profile slot to use
         * @return {@link RunCommand}
         */
        public Command setTargetDistanceDynamic(Distance distance,
                LinearVelocity mmVelocity,
                LinearAcceleration mmAccel,
                int slotId)
        {
            return setTargetDistanceDynamic(distance, mmVelocity, mmAccel, slotId);
        }

        /**
         * Create an angular velocity command. Continuously updates the target while running
         * 
         * @param velocitySupplier
         *            A supplier that provides the target angular velocity
         * @return {@link RunCommand}
         */
        public Command setTargetAngularVelocity(Supplier<AngularVelocity> velocitySupplier)
        {
            return run(() ->
            {
                servoMotor.setTargetVelocity(velocitySupplier.get());
            }).withName(getName() + " Velocity");
        }

        /**
         * Create an angular velocity command.
         * 
         * @param velocity
         *            The target angular velocity
         * @return {@link RunCommand}
         */
        public Command setTargetLinearVelocity(Supplier<LinearVelocity> velocitySupplier)
        {
            return run(() ->
            {
                servoMotor.setTargetLinearVelocity(velocitySupplier.get());
            }).withName(getName() + " Velocity");
        }

        /**
         * Create a command to set the output to 0%, using the currently configured Brake mode {@link NeutralModeValue}
         * 
         * @return
         */
        public Command setTargetNeutral()
        {
            return Commands.run(() -> servoMotor.setTargetNeutral())
                    .ignoringDisable(true);
        }

        /**
         * Create a command to set the output to 0%, with coast mode enabled
         * 
         * @return
         */
        public Command setTargetCoast()
        {
            return Commands.run(() -> servoMotor.setTargetCoast())
                    .ignoringDisable(true);
        }

        /**
         * Set the motor in coast mode until the command ends
         */
        public Command setConfigCoast()
        {
            return Commands.startEnd(() -> servoMotor.setConfigNeutralMode(NeutralModeValue.Coast),
                    () -> servoMotor.setConfigNeutralMode(NeutralModeValue.Brake)).withName(getName() + " CoastMode")
                    .ignoringDisable(true);
        }

        public Command setTargetVoltage(Supplier<Voltage> volts)
        {
            return Commands.run(() -> servoMotor.setTargetVoltage(volts.get()))
                    .withName(getName() + " Voltage");
        }

        /**
         * Create a command that waits until close to the target position
         */
        public Command waitForAngle(Supplier<Angle> targetPosition, Angle tolerance)
        {
            return new WaitUntilCommand(
                    () -> Util.epsilonEquals(
                            getActuals().getAngle().in(Degree),
                            targetPosition.get().in(Degree),
                            tolerance.in(Degree)));
        }

        /**
         * Create a command that disables soft limits at the start, and then re-sets them to their previous config on end.
         * Use this in a command group to temporarily bypass soft limits
         * 
         * @return {@link StartEndCommand}
         */
        public Command withoutSoftLimitsTemporarily()
        {
            var prev = new Object()
            {
                boolean fwd = false;
                boolean rev = false;
            };
            return Commands.startEnd(() ->
            {
                SoftwareLimitSwitchConfigs swConfig = servoMotor.getSoftwareLimitSwitchConfigs();
                prev.fwd = swConfig.ForwardSoftLimitEnable;
                prev.rev = swConfig.ReverseSoftLimitEnable;
                servoMotor.setConfigSoftLimitsEnable(false, false);
            }, () ->
            {
                servoMotor.setConfigSoftLimitsEnable(prev.fwd, prev.rev);
            });
        }

        /**
         * Create a command that disables current limits at the start, and then re-sets them to their previous config on
         * end. Use this in a command group to temporarily bypass current limits
         * 
         * @return {@link StartEndCommand}
         */
        public Command withoutCurrentLimitsTemporarily()
        {
            var prev = new Object()
            {
                boolean statorLimits = false;
                boolean supplyLimits = false;
            };
            return Commands.startEnd(() ->
            {
                CurrentLimitsConfigs currentLimitsConfigs = servoMotor.getCurrentLimitsConfigs();
                prev.statorLimits = currentLimitsConfigs.StatorCurrentLimitEnable;
                prev.supplyLimits = currentLimitsConfigs.SupplyCurrentLimitEnable;
                servoMotor.setConfigCurrentLimitsEnable(false, false);
            }, () ->
            {
                servoMotor.setConfigCurrentLimitsEnable(prev.statorLimits, prev.supplyLimits);
            });
        }

        /**
         * Create a command that moves a roller a certain distance, relative to current position
         * 
         * @param angleDelta
         *            The change in angle to move, relative to current position
         * @param tolerance
         *            The tolerance to consider "at target"
         * @return {@link SequentialCommandGroup}
         */
        public Command setRelativeAngle_UntilAtAngle(Angle angleDelta, Angle tolerance)
        {
            var targetPosition = new Object()
            {
                Angle rotations;
            };
            return new SequentialCommandGroup(
                    Commands.runOnce(() ->
                    {
                        targetPosition.rotations = getActuals().getAngle().plus(angleDelta);
                    }),
                    setTargetAngle(() -> targetPosition.rotations)
                            .until(triggers.atTargetPosition(tolerance)));
        }

        /**
         * Create a command that moves a roller a certain distance, relative to current position
         * 
         * @param distanceDelta
         *            The change in distance to move, relative to current position
         * @param tolerance
         *            The tolerance to consider "at target"
         * @return {@link SequentialCommandGroup}
         */
        public Command setRelativeDistance_UntilAtDistance(Distance distanceDelta, Distance tolerance)
        {
            var targetPosition = new Object()
            {
                Distance newPosition;
            };
            return new SequentialCommandGroup(
                    Commands.runOnce(() ->
                    {
                        targetPosition.newPosition = getActuals().getDistance().plus(
                                distanceDelta);
                    }),
                    setTargetDistance(() -> targetPosition.newPosition)
                            .until(triggers.atTargetPosition(tolerance)));
        }

        /**
         * Create a command that sets a custom ControlRequest. Use this for advanced control of the motor that isn't covered
         * by the other commands, like modifying the request to disable soft or current limits temporarily.
         * 
         * @param controlRequest
         *            The ControlRequest to set while this command is running
         * @return {@link RunCommand}
         */
        public Command setCustomRequest(ControlRequest controlRequest)
        {
            return run(() ->
            {
                servoMotor.setCustomRequest(controlRequest);
            }).withName(getName() + " CustomRequestControl");
        }
    }

    /**
     * Get the generic triggers for a servo motor
     * 
     * @return ServoMotorTriggers
     */
    public ServoMotorTriggers getServoTriggers()
    {
        return triggers;
    }

    /**
     * Set of generic triggers for a servo motor. Building blocks that can be added onto within an implementation of a
     * ServoMotorSubsystem
     */
    public class ServoMotorTriggers
    {

        public Trigger atTargetPosition(Angle tolerance)
        {
            return new Trigger(
                    () -> Util.epsilonEquals(
                            getActuals().getAngle().in(Degree),
                            getTargets().getAngle().in(Degree),
                            tolerance.in(Degree)));
        }

        public Trigger atTargetPosition(Distance tolerance)
        {
            return new Trigger(
                    () -> Util.epsilonEquals(
                            getActuals().getDistance().in(Meters),
                            getTargets().getDistance().in(Meters),
                            tolerance.in(Meters)));
        }

        public Trigger atTargetVelocity(AngularVelocity tolerance)
        {
            return new Trigger(
                    () -> Util.epsilonEquals(
                            getActuals().getAngularVelocity().in(RotationsPerSecond),
                            getTargets().getAngularVelocity().in(RotationsPerSecond),
                            tolerance.in(RotationsPerSecond)));
        }

        public Trigger atTargetVelocity(LinearVelocity tolerance)
        {
            return new Trigger(
                    () -> Util.epsilonEquals(
                            getActuals().getLinearVelocity().in(MetersPerSecond),
                            getTargets().getLinearVelocity().in(MetersPerSecond),
                            tolerance.in(MetersPerSecond)));
        }

        public Trigger atSoftLimitForward()
        {
            return new Trigger(
                    () -> isAtForwardLimit());
        }

        public Trigger atSoftLimitReverse()
        {
            return new Trigger(
                    () -> isAtReverseLimit());
        }

        // public Trigger isStalled()
    }

    /**
     * Use the current position as the zero calibration offset
     */
    public void setCurrentPositionAsZero()
    {
        servoMotor.setConfigCurrentPositionAsZero();
    }

    /**
     * Set the current position to a specific value
     * 
     * @param positionUnits
     *            The position to set the current position to
     */
    public void setCurrentPosition(Angle positionUnits)
    {
        servoMotor.setConfigCurrentPosition(positionUnits);
    }

    /**
     * Check if the motor is at the forward soft limit
     * 
     * @return
     */
    public boolean isAtForwardLimit()
    {
        return getActuals().softLimitForward;
    }

    /**
     * Check if the motor is at the reverse soft limit
     * 
     * @return
     */
    public boolean isAtReverseLimit()
    {
        return getActuals().softLimitReverse;
    }

    /***
     * Sys ID Functions
     */
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    null, // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> getServoMotor().setTargetVoltage(volts),
                    null,
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
    {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction)
    {
        return sysIdRoutine.dynamic(direction);
    }
}
