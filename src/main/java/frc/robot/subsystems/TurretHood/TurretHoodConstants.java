package frc.robot.subsystems.TurretHood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.motor.StatusSignalTypes;
import frc.lib.servomotor.ServoMotorConfig.ServoMotorConfigFX;
import frc.lib.servomotor.motors.ServoMotorTalonFX;
import frc.lib.servomotor.motors.sim.ServoMotorTalonFXSim;
import frc.lib.util.CANDeviceId;
import frc.lib.util.UnitsMath;
import frc.robot.configuration.Constants;

public class TurretHoodConstants
{

    public static final String NAME = "TurretHood";

    public static final CANDeviceId TALON_CAN_ID = Constants.CAN.TURRETHOOD;

    public static final Angle REVERSE_MIN_POSITION = Degrees.of(20); // 70 shooting degrees
    public static final Angle FORWARD_MAX_POSITION = Degrees.of(45); // 50 shooting degrees
    public static final Angle POSITION_TOLERANCE = Degrees.of(1.5);

    public static final Angle UNDER_TRENCH_ANGLE = Degrees.of(20.5); // 69.5 shooting degrees

    // Imagine 90 is shooter facing the wall dead on, its for easy command making
    public static final Angle MAX_SHOT_ANGLE = HoodAngleToShotAngle(FORWARD_MAX_POSITION);
    public static final Angle MIN_SHOT_ANGLE = HoodAngleToShotAngle(REVERSE_MIN_POSITION);
    public static final Distance MIN_SHOOTING_DISTANCE = Meters.of(0.5);
    public static final Distance MAX_SHOOTING_DISTANCE = Meters.of(12);// OG 5, 6.3001820656 is mesured from the center of goal
                                                                       // to the edge near depot / human player

    // max/min shot angles
    public static final Angle MIN_DIS_SHOT_ANGLE = Degrees.of(69.5);
    public static final Angle MAX_DIS_SHOT_ANGLE = Degrees.of(30.5); // TODO: measure this

    public static final double GEAR_RATIO = ((36.0 / 9.0) * (18.0 / 16.0) * (294.0 / 20.0));

    // =============VIZ===============
    public static final Distance HOOD_LENGTH = Inches.of(9);

    public static final Mass CG_MASS = Pound.of(5);
    public static final Distance CG_DISTANCE = Inches.of(9);
    public static final MomentOfInertia MOMENT_OF_INERTIA = UnitsMath.CalcMomentOfInertia(CG_MASS, CG_DISTANCE);

    public static final Pose3d ORIGIN = new Pose3d(Inches.of(0), Inches.of(0), Inches.of(20), new Rotation3d());
    // =============VIZ===============

    public static Angle ShotAngleToHoodAngle(Angle shotAngle)
    {
        return Degrees.of(90).minus(shotAngle);
    }

    public static Angle HoodAngleToShotAngle(Angle hoodAngle)
    {
        return Degrees.of(90).minus(hoodAngle);
    }

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
        ServoMotorConfigFX config = new ServoMotorConfigFX(NAME, TALON_CAN_ID)
                .withGearRatio(GEAR_RATIO)
                .withSoftLimits(REVERSE_MIN_POSITION, FORWARD_MAX_POSITION);

        // Actual Robot v
        config.motorConfig.Slot0.kP = 450.0;
        config.motorConfig.Slot0.kD = 0.0;
        config.motorConfig.Slot0.kS = 0.5;
        config.motorConfig.Slot0.kV = 2.5;
        config.motorConfig.Slot0.kG = 0.0;
        // This tune makes the turret .5 deg off

        // config.motorConfig.Slot0.kP = 19.0; //This is for SimRobot
        // config.motorConfig.Slot0.kD = 10.0; //This is for SimRobot
        config.motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.motorConfig.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(6));
        config.motorConfig.MotionMagic.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10));

        // Fix the orientation
        config.motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.momentOfInertia = MOMENT_OF_INERTIA;

        config.signalTypes = StatusSignalTypes.All;

        return config;
    }
}
