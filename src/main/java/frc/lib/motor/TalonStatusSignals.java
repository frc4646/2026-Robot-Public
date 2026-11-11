package frc.lib.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.DistanceAngleConverter;

/**
 * Container class to hold the {@link StatusSignal}s for the Talons
 */
public class TalonStatusSignals
{
    public StatusSignalTypes signalTypes = StatusSignalTypes.All;

    protected StatusSignal<Double> dutyCycleSignal;
    protected StatusSignal<Angle> positionSignal;
    protected StatusSignal<AngularVelocity> velocitySignal;
    protected StatusSignal<Voltage> voltageSignal;
    protected StatusSignal<Current> currentStatorSignal;
    protected StatusSignal<Current> currentSupplySignal;
    protected StatusSignal<Current> torqueCurrentSignal;
    protected StatusSignal<Boolean> forwardSoftLimit;
    protected StatusSignal<Boolean> reverseSoftLimit;

    protected final BaseStatusSignal[] signals;

    /**
     * Constructor for Signals that initializes the status signals from a TalonFX.
     *
     * @param talon
     *            the TalonFX motor controller to get the signals from
     */
    public TalonStatusSignals(TalonFX talon, StatusSignalTypes signalTypes)
    {
        this.dutyCycleSignal = talon.getDutyCycle();
        this.positionSignal = talon.getPosition();
        this.velocitySignal = talon.getVelocity();
        this.voltageSignal = talon.getMotorVoltage();
        this.currentStatorSignal = talon.getStatorCurrent();
        this.currentSupplySignal = talon.getSupplyCurrent();
        this.torqueCurrentSignal = talon.getTorqueCurrent();
        this.forwardSoftLimit = talon.getFault_ForwardSoftLimit();
        this.reverseSoftLimit = talon.getFault_ReverseSoftLimit();

        this.signalTypes = signalTypes;

        signals = setupSignalsFromType();
    }

    /**
     * Constructor for Signals that initializes the status signals from a TalonFXS.
     *
     * @param talon
     *            the TalonFXS motor controller to get the signals from
     */
    public TalonStatusSignals(TalonFXS talon, StatusSignalTypes signalTypes)
    {
        this.dutyCycleSignal = talon.getDutyCycle();
        this.positionSignal = talon.getPosition();
        this.velocitySignal = talon.getVelocity();
        this.voltageSignal = talon.getMotorVoltage();
        this.currentStatorSignal = talon.getStatorCurrent();
        this.currentSupplySignal = talon.getSupplyCurrent();
        this.torqueCurrentSignal = talon.getTorqueCurrent();
        this.forwardSoftLimit = talon.getFault_ForwardSoftLimit();
        this.reverseSoftLimit = talon.getFault_ReverseSoftLimit();

        this.signalTypes = signalTypes;

        signals = setupSignalsFromType();
    }

    private BaseStatusSignal[] setupSignalsFromType()
    {
        switch (signalTypes)
        {
            case Currents:
                return new BaseStatusSignal[]
                {
                        voltageSignal,
                        currentStatorSignal,
                        currentSupplySignal,
                        torqueCurrentSignal
                };
            case CurrentsAndVelocity:
                return new BaseStatusSignal[]
                {
                        velocitySignal,
                        voltageSignal,
                        currentStatorSignal,
                        currentSupplySignal,
                        torqueCurrentSignal
                };
            case CurrentsAndPosition:
                return new BaseStatusSignal[]
                {
                        positionSignal,
                        voltageSignal,
                        currentStatorSignal,
                        currentSupplySignal,
                        torqueCurrentSignal,
                        forwardSoftLimit,
                        reverseSoftLimit
                };
            case CurrentsAndDutyCycle:
                return new BaseStatusSignal[]
                {
                        dutyCycleSignal,
                        voltageSignal,
                        currentStatorSignal,
                        currentSupplySignal,
                        torqueCurrentSignal
                };
            case All:
            default:
                return new BaseStatusSignal[]
                {
                        dutyCycleSignal,
                        positionSignal,
                        velocitySignal,
                        voltageSignal,
                        currentStatorSignal,
                        currentSupplySignal,
                        torqueCurrentSignal,
                        forwardSoftLimit,
                        reverseSoftLimit
                };
        }
    }

    /**
     * Gets the array of {@link BaseStatusSignal} for the TalonFX.
     * 
     * @return an array of BaseStatusSignal containing the status signals
     */
    public BaseStatusSignal[] get()
    {
        return signals;
    }

    /**
     * Helper function that updates the sensors from the signals.
     *
     * @param sensors
     *            the MotorSensorsAutoLogged instance to update
     * @param signals
     *            the Signals instance containing the status signals
     */
    public void updateSensorsFromSignals(MotorActualsAutoLogged sensors, DistanceAngleConverter converter)
    {
        if (signalTypes == StatusSignalTypes.CurrentsAndDutyCycle || signalTypes == StatusSignalTypes.All)
        {
            sensors.dutyCycle = dutyCycleSignal.getValueAsDouble();
        }

        if (signalTypes == StatusSignalTypes.CurrentsAndPosition || signalTypes == StatusSignalTypes.All)
        {
            Angle position = positionSignal.getValue();
            sensors.positionRotations = position.in(Rotations);
            sensors.positionInches = converter.toDistance(position).in(Inches);

            sensors.softLimitForward = forwardSoftLimit.getValue();
            sensors.softLimitReverse = reverseSoftLimit.getValue();
        }

        if (signalTypes == StatusSignalTypes.CurrentsAndVelocity || signalTypes == StatusSignalTypes.All)
        {
            AngularVelocity velocity = velocitySignal.getValue();
            sensors.velocityRotationsPerSecond = velocity.in(RotationsPerSecond);
            sensors.velocityInchesPerSecond = converter.toLinearVelocity(velocity).in(InchesPerSecond);
        }

        sensors.appliedVolts = voltageSignal.getValue().in(Volts);
        sensors.currentStatorAmps = currentStatorSignal.getValue().in(Amps);
        sensors.currentSupplyAmps = currentSupplySignal.getValue().in(Amps);
        sensors.torqueCurrentAmps = torqueCurrentSignal.getValue().in(Amps);

    }

    /**
     * Creates a minimal TalonStatusSignals for follower motors (currents only).
     *
     * @param talon
     *            the follower TalonFX motor controller
     * @return TalonStatusSignals configured for currents only
     */
    public static TalonStatusSignals forFollower(TalonFX talon)
    {
        return new TalonStatusSignals(talon, StatusSignalTypes.Currents);
    }

    /**
     * Creates a minimal TalonStatusSignals for follower motors (currents only).
     *
     * @param talon
     *            the follower TalonFXS motor controller
     * @return TalonStatusSignals configured for currents only
     */
    public static TalonStatusSignals forFollower(TalonFXS talon)
    {
        return new TalonStatusSignals(talon, StatusSignalTypes.Currents);
    }

}