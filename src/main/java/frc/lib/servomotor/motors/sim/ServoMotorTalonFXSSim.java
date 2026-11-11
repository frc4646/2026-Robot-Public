package frc.lib.servomotor.motors.sim;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.servomotor.ServoMotorConfig.ServoMotorConfigFXS;
import frc.lib.servomotor.motors.ServoMotorTalonFXS;
import frc.lib.time.RobotTime;

public class ServoMotorTalonFXSSim extends ServoMotorTalonFXS
{
    private final String name;
    private final DCMotor motor;

    protected DCMotorSim motorSim;

    private Notifier simNotifier = null;
    private double lastUpdateTimestamp = 0.0;

    private double ratio;

    public ServoMotorTalonFXSSim(ServoMotorConfigFXS config)
    {
        super(config);
        this.name = config.name;

        int motorCount = config.followerId.isPresent() ? 2 : 1;
        motor = DCMotor.getKrakenX60Foc(motorCount);

        motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        motor,
                        config.momentOfInertia.in(KilogramSquareMeters),
                        config.motorConfig.ExternalFeedback.RotorToSensorRatio),
                motor);

        ratio = config.motorConfig.ExternalFeedback.SensorToMechanismRatio;

        // Assume that config is correct (which it might not be)
        talon.getSimState().ExtSensorOrientation = (config.motorConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
                ? ChassisReference.CounterClockwise_Positive
                : ChassisReference.Clockwise_Positive;

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() ->
        {
            updateSimState();
        });
        simNotifier.startPeriodic(0.005);

    }

    protected double addFriction(double motorVoltage, double frictionVoltage)
    {
        if (Math.abs(motorVoltage) < frictionVoltage)
        {
            motorVoltage = 0.0;
        }
        else if (motorVoltage > 0.0)
        {
            motorVoltage -= frictionVoltage;
        }
        else
        {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }

    protected void updateSimState()
    {
        // apply the voltage to the sim
        // double simVoltage = addFriction(talon.getSimState().getMotorVoltage(), 0.25);
        var voltage = talon.getSimState().getMotorVoltage();
        motorSim.setInput(voltage);

        // update the sim
        double timestamp = RobotTime.getTimestampSeconds();
        motorSim.update(timestamp - lastUpdateTimestamp);
        lastUpdateTimestamp = timestamp;

        // update position
        var position = Radians.of(motorSim.getAngularPositionRad()).times(ratio);
        talon.getSimState().setRawRotorPosition(position);

        // update velocity
        var velocity = RadiansPerSecond.of(motorSim.getAngularVelocityRadPerSec()).times(ratio);
        talon.getSimState().setRotorVelocity(velocity);

        Logger.recordOutput(name + "/Sim/AppliedVolts", voltage);
        Logger.recordOutput(name + "/Sim/PositionRotations", position.in(Rotations));
        Logger.recordOutput(name + "/Sim/VelocityRotationsPerSecond", velocity.in(RotationsPerSecond));

        if (cancoder.isPresent())
        {
            cancoder.get().getSimState().setRawPosition(position);
        }
    }

}
