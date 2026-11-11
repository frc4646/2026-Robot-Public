package frc.lib.factories;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * Factory class for creating various {@link ControlRequest} objects.
 * This class provides static methods to create requests for duty cycle, voltage, position, velocity,
 * and dynamic position control with optional slot IDs.
 */
public class ControlRequestFactory
{
    private ControlRequestFactory()
    {
    }

    /**
     * Creates a {@link NeutralOut} request (0% output) using the currently configured Brake mode {@link NeutralModeValue}
     * 
     * @return a ControlRequest that sets output to 0
     */
    public static NeutralOut withNeutral()
    {
        return new NeutralOut();
    }

    /**
     * Creates a {@link CoastOut} request (0% output) with coast mode enabled
     * 
     * @return a ControlRequest that sets output to 0 and applies coast mode
     */
    public static CoastOut withCoast()
    {
        return new CoastOut();
    }

    /**
     * Creates a {@link DutyCycleOut} request. FOC is enabled.
     * 
     * @param percent
     *            The duty cycle percentage to set (-1.0 to 1.0).
     * @return A ControlRequest that sets the specified duty cycle percentage with FOC enabled.
     */
    public static DutyCycleOut withDutyCycle(double percent)
    {
        return new DutyCycleOut(percent).withEnableFOC(true);
    }

    /**
     * Creates a {@link VoltageOut} request. FOC is enabled.
     * 
     * @param voltage
     *            The voltage to set.
     * @return A ControlRequest that sets the specified voltage with FOC enabled.
     */
    public static VoltageOut withVoltage(Voltage voltage)
    {
        return new VoltageOut(voltage).withEnableFOC(true);
    }

    /**
     * Creates a {@link MotionMagicVoltage} position control request. FOC is enabled.
     * 
     * @param voltage
     *            The voltage to set.
     * @return A ControlRequest that sets the specified voltage with FOC enabled.
     */
    public static MotionMagicVoltage withPosition(Angle mechanismPosition)
    {
        // return new PositionVoltage(mechanismPosition).withEnableFOC(true);
        // return new MotionMagicExpoVoltage(mechanismPosition).withEnableFOC(true);
        return new MotionMagicVoltage(mechanismPosition).withEnableFOC(true);
    }

    /**
     * Creates a {@link VelocityTorqueCurrentFOC} request for velocity control. FOC is enabled.
     * 
     * @param mechanismVelocity
     *            The desired velocity to set.
     * @return A ControlRequest that sets the specified velocity with FOC enabled.
     */
    public static VelocityTorqueCurrentFOC withVelocity(AngularVelocity mechanismVelocity)
    {
        return new VelocityTorqueCurrentFOC(mechanismVelocity);
    }

    /**
     * Creates a {@link DynamicMotionMagicVoltage} request for position control with custom max {@link AngularVelocity} and
     * {@link AngularAcceleration} limits. FOC is enabled.
     * 
     * @param position
     *            The desired position to set.
     * @param velocity
     *            The max velocity to set.
     * @param acceleration
     *            The max acceleration to set.
     * @return A ControlRequest that sets the specified position, velocity, and acceleration with FOC enabled.
     */
    public static DynamicMotionMagicVoltage withDynamicPosition(Angle position, AngularVelocity velocity, AngularAcceleration acceleration)
    {
        return new DynamicMotionMagicVoltage(position, velocity, acceleration)
                .withEnableFOC(true);
    }

}
