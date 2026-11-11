// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeRollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.motor.StatusSignalTypes;
import frc.lib.servomotor.ServoMotorConfig.ServoMotorConfigFX;
import frc.lib.servomotor.motors.ServoMotorTalonFX;
import frc.lib.servomotor.motors.sim.ServoMotorTalonFXSim;
import frc.lib.util.CANDeviceId;
import frc.robot.configuration.Constants;

/** Add your docs here. */
public class IntakeRollerConstants
{
    public static final String NAME = "IntakeRollers";

    public static final CANDeviceId TALON_CAN_ID = Constants.CAN.INTAKE_ROLLERS;
    // public static final CANDeviceId CANCODER_CAN_ID = Constants.CAN.INTAKE_CANCODER;
    public static final CANDeviceId FOLLOWER_ID = Constants.CAN.INTAKE_ROLLERS_FOLLOWER;
    public static final Angle ZERO_POSITION_OFFSET = Rotations.of(0.255); // TODO find from Phoenix Tuner

    public static final AngularVelocity INTAKE_SPIN_SPEED_AUTO = RPM.of(4800); // figure out the speed of this
    public static final double INTAKE_STOP_SPEED_AUTO = 0.0;
    public static final double INTAKE_EJECT_SPEED_AUTO = -0.7;

    public static final Voltage INTAKE_SPIN_SPEED_TELE = Volts.of(.7 * 12); // figure out the speed of this
    public static final Voltage INTAKE_STOP_SPEED_TELE = Volts.of(0.0 * 12);
    public static final Voltage INTAKE_EJECT_SPEED_TELE = Volts.of(-0.7 * 12);

    protected static ServoMotorTalonFX getServoMotor()
    {
        if (RobotBase.isReal())
        {
            return new ServoMotorTalonFX(getServoConfig());
        }
        else
        {
            return new ServoMotorTalonFXSim(getServoConfig());
        }
    }

    private static ServoMotorConfigFX getServoConfig()
    {
        ServoMotorConfigFX config = new ServoMotorConfigFX(NAME, TALON_CAN_ID).withFollower(FOLLOWER_ID, InvertedValue.Clockwise_Positive);

        config.motorConfig.Slot0.kP = 6.0;
        config.motorConfig.Slot0.kS = 3.0;
        config.motorConfig.Slot0.kV = 0.01;
        config.motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        config.motorConfig.CurrentLimits.StatorCurrentLimit = 100;
        config.motorConfig.CurrentLimits.SupplyCurrentLimit = 33;
        config.motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 33;
        config.motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.motorConfig.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(100));
        config.motorConfig.MotionMagic.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(500));
        config.motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // Fix the orientation
        config.motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // config.momentOfInertia = MOMENT_OF_INERTIA;

        config.signalTypes = StatusSignalTypes.CurrentsAndVelocity;

        return config;
    }

}
