package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.motor.StatusSignalTypes;
import frc.lib.servomotor.ServoMotorConfig.ServoMotorConfigFX;
import frc.lib.servomotor.motors.ServoMotorTalonFX;
import frc.lib.servomotor.motors.sim.ServoMotorTalonFXSim;
import frc.lib.util.CANDeviceId;
import frc.robot.configuration.Constants;

public class IndexerConstants
{

    public static final String SPINDEXER_NAME = "Indexer Spindexer";
    public static final String BELT_WALL_NAME = "Indexer Belt Wall";
    public static final String FEEDER_NAME = "Indexer Feeder";

    public static final CANDeviceId SPINDEXER_CAN_ID = Constants.CAN.SPINDEXER;
    public static final CANDeviceId BELT_WALL_CAN_ID = Constants.CAN.BELT_WALL;
    public static final CANDeviceId BELT_WALL_FOLLOWER_CAN_ID = Constants.CAN.BELT_WALL_FOLLOWER;

    public static final CANDeviceId FEEDER_CAN_ID = Constants.CAN.FEEDER;

    public static final double SPINDEXER_SPEED_REDBEAN = 1.0; // TODO tune :D
    public static final double BELT_WALL_SPEED_REDBEAN = 1.0;
    public static final double FEEDER_SPEED_REDBEAN = 0.7;

    public static final double SPINDEXER_SPEED_BLACKBEARD = 0.7;
    public static final double FEEDER_SPEED_BLACKBEARD = 1.0;

    public static final double SHOOTING_TIMEOUT = 0.75;

    public static final Distance NO_FUEL_DISTANCE = Inches.of(4); // TODO !!!!! tuneeeee
    public static final double FeederToSpindexer = 1.0; // 1:1

    public static final double SpindexerStatorCurrentLimit = 40.0;
    public static final double BeltWallStatorCurrentLimit = 60.0;
    public static final double FeederStatorCurrentLimit = 45.0;

    public static final double SPINDEXER_GEAR_RATIO = (5.0 / 1.0) * (3.0 / 1.0);
    public static final double BELT_WALL_GEAR_RATIO = (22.0 / 18.0); // * (5.0 / 2.0); // do we need this extra step for belt wall?
    public static final double FEEDER_GEAR_RATIO = (48.0 / 32.0) * (27.0 / 27.0);

    public static final AngularVelocity SPINDEXER_FEED_VELOCITY = RPM.of(320.0); // TODO tune;
    public static final AngularVelocity BELT_WALL_FEED_VELOCITY = RPM.of(3230.0); // TODO tune;
    public static final AngularVelocity FEEDER_FEED_VELOCITY = RPM.of(1500.0); // TODO tune;

    public static final Distance SPINDEXER_RADIUS = Inches.of(4.5); // TODO tune
    public static final Distance BELT_WALL_RADIUS = Inches.of(10.0); // TODO tune
    public static final Distance FEEDER_RADIUS = Inches.of(3.5); // TODO tune

    private static ServoMotorConfigFX getServoConfigFeeder()
    {
        ServoMotorConfigFX config = new ServoMotorConfigFX(FEEDER_NAME, FEEDER_CAN_ID, FEEDER_RADIUS)
                .withGearRatio(FEEDER_GEAR_RATIO);

        config.motorConfig.Slot0.kP = 20.0;
        config.motorConfig.Slot0.kV = 0.01;
        config.motorConfig.Slot0.kS = 8.0;

        config.motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        config.motorConfig.CurrentLimits.StatorCurrentLimit = FeederStatorCurrentLimit;
        config.motorConfig.CurrentLimits.SupplyCurrentLimit = 30;
        config.motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;

        config.motorConfig.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(1));
        config.motorConfig.MotionMagic.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5));

        // Fix the orientation
        config.motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.signalTypes = StatusSignalTypes.CurrentsAndVelocity;

        return config;
    }

    protected static ServoMotorTalonFX getServoMotorFeeder()
    {
        if (RobotBase.isReal())
        {
            return new ServoMotorTalonFX(getServoConfigFeeder());
        }
        else
        {
            return new ServoMotorTalonFXSim(getServoConfigFeeder());
        }
    }

    private static ServoMotorConfigFX getServoConfigSPINDEXER()
    {
        ServoMotorConfigFX config = new ServoMotorConfigFX(SPINDEXER_NAME, SPINDEXER_CAN_ID, SPINDEXER_RADIUS)
                .withGearRatio(SPINDEXER_GEAR_RATIO);

        config.motorConfig.Slot0.kP = 5.0;
        config.motorConfig.Slot0.kV = 1.15;
        config.motorConfig.Slot0.kS = 4.0;

        config.motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        config.motorConfig.CurrentLimits.StatorCurrentLimit = SpindexerStatorCurrentLimit;
        config.motorConfig.CurrentLimits.SupplyCurrentLimit = 24;
        config.motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 24;

        config.motorConfig.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(1));
        config.motorConfig.MotionMagic.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5));

        // Fix the orientation
        config.motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.signalTypes = StatusSignalTypes.CurrentsAndVelocity;

        return config;
    }

    protected static ServoMotorTalonFX getServoMotorSPINDEXER()
    {
        if (RobotBase.isReal())
        {
            return new ServoMotorTalonFX(getServoConfigSPINDEXER());
        }
        else
        {
            return new ServoMotorTalonFXSim(getServoConfigSPINDEXER());
        }
    }

    private static ServoMotorConfigFX getServoConfigBELTWALL()
    {
        ServoMotorConfigFX config = new ServoMotorConfigFX(BELT_WALL_NAME, BELT_WALL_CAN_ID, BELT_WALL_RADIUS)
                .withGearRatio(BELT_WALL_GEAR_RATIO)
                .withFollower(BELT_WALL_FOLLOWER_CAN_ID, InvertedValue.CounterClockwise_Positive);

        config.motorConfig.Slot0.kS = 25.0;
        config.motorConfig.Slot0.kV = 0.75;
        config.motorConfig.Slot0.kP = 7.0;

        config.motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        config.motorConfig.CurrentLimits.StatorCurrentLimit = BeltWallStatorCurrentLimit;
        config.motorConfig.CurrentLimits.SupplyCurrentLimit = 30;
        config.motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;

        config.motorConfig.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(1));
        config.motorConfig.MotionMagic.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5));

        // Fix the orientation
        config.motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.signalTypes = StatusSignalTypes.CurrentsAndVelocity;

        return config;
    }

    protected static ServoMotorTalonFX getServoMotorBELTWALL()
    {
        if (RobotBase.isReal())
        {
            return new ServoMotorTalonFX(getServoConfigBELTWALL());
        }
        else
        {
            return new ServoMotorTalonFXSim(getServoConfigBELTWALL());
        }
    }
}
