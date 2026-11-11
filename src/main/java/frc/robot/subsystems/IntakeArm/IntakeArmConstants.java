// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeArm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.factories.CANcoderFactory;
import frc.lib.motor.StatusSignalTypes;
import frc.lib.servomotor.ServoMotorConfig.ServoMotorConfigFX;
import frc.lib.servomotor.motors.ServoMotorTalonFX;
import frc.lib.servomotor.motors.sim.ServoMotorTalonFXSim;
import frc.lib.util.CANDeviceId;
import frc.lib.util.UnitsMath;
import frc.robot.configuration.Constants;

/** Add your docs here. */
public class IntakeArmConstants
{
    public static final String NAME = "Intake Arm";

    public static final CANDeviceId TALON_CAN_ID = Constants.CAN.INTAKE;
    public static final CANDeviceId TALON_FOLLOWER_CAN_ID = Constants.CAN.INTAKE_FOLLOWER;
    public static final CANDeviceId CANCODER_CAN_ID = Constants.CAN.INTAKE_CANCODER;

    public static final Angle REVERSE_MIN_POSITION = Rotations.of(0.12);
    public static final Angle FORWARD_MAX_POSITION = Degrees.of(180);

    public static final Angle ZERO_POSITION_OFFSET = Rotations.of(.3141592653); // tune

    public static final double GEAR_RATIO = 23.0;

    public static final Distance ARM_LENGTH = Inches.of(5.0);

    public static final Pose3d ORIGIN = new Pose3d(
            Inches.of(-12),
            Inches.of(0),
            Inches.of(12),
            new Rotation3d(Degrees.zero(), Degrees.of(0), Degrees.zero()));

    public static final Mass CG_MASS = Pounds.of(5);
    public static final Distance CG_DISTANCE = Inches.of(9);
    public static final MomentOfInertia MOMENT_OF_INERTIA = UnitsMath.CalcMomentOfInertia(CG_MASS, CG_DISTANCE);

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

    private static CANcoderConfiguration getCANCoderConfig()
    {
        CANcoderConfiguration config = CANcoderFactory.makeDefaultConfig(ZERO_POSITION_OFFSET);
        // Fix the orientation
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.75;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        return config;
    }

    private static ServoMotorConfigFX getServoConfig()
    {
        ServoMotorConfigFX config = new ServoMotorConfigFX(NAME, TALON_CAN_ID)
                .withCANcoder(CANCODER_CAN_ID, getCANCoderConfig(), GEAR_RATIO, 1.0)
                .withSoftLimits(REVERSE_MIN_POSITION, FORWARD_MAX_POSITION)
                .withFollower(TALON_FOLLOWER_CAN_ID, InvertedValue.Clockwise_Positive);

        config.motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        config.motorConfig.CurrentLimits.StatorCurrentLimit = 55;
        config.motorConfig.CurrentLimits.SupplyCurrentLimit = 30;
        config.motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;

        config.motorConfig.Slot0.kP = 5.0;
        config.motorConfig.Slot0.kS = 0.75;
        config.motorConfig.Slot0.kV = 1.75;

        config.motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.motorConfig.Slot1.kP = 7.0;
        config.motorConfig.Slot1.kS = 0.75;
        config.motorConfig.Slot1.kV = 2.25;

        config.motorConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;

        config.motorConfig.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(1));
        config.motorConfig.MotionMagic.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5));

        // Fix the orientation
        config.motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.momentOfInertia = MOMENT_OF_INERTIA;

        config.signalTypes = StatusSignalTypes.CurrentsAndPosition;

        return config;
    }
}
