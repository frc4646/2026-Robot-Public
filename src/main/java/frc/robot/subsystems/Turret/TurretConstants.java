package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
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
import frc.robot.configuration.Constants;

public class TurretConstants
{

    public static final String NAME = "Turret";

    public static final CANDeviceId TALON_CAN_ID = Constants.CAN.TURRET;
    public static final CANDeviceId CANCODER_ID = Constants.CAN.TURRET_CANCODER;

    public static final Angle MIN_ANGLE = Degrees.of(-210);
    public static final Angle MAX_ANGLE = Degrees.of(210);

    public static final double GEAR_RATIO = (40.0 / 14.0) * (144.0 / 22.0);
    public static final double CANCODER_GEAR_RATIO = 144.0 / 24.0;

    public static final double FEED_FORWARD_RATIO = 0.04;

    // Calibration offsets: sensor readings when turret is at true zero
    // To calibrate: move turret to desired zero position, record both sensor values
    public static final Angle MOTOR_ZERO_OFFSET = Rotations.of(.06445).times(GEAR_RATIO);
    public static final Angle CANCODER_ZERO_OFFSET = Rotations.of(.0986);
    // Enable this to get live calculations on the solve algorithm
    public static final boolean DEBUG_POSITIONS = true;

    static final double TURRET_MAX_ROTATIONS = (MAX_ANGLE.in(Rotations) - MIN_ANGLE.in(Rotations));
    static final int MAX_MOTOR_WRAPS = (int) Math.ceil(TURRET_MAX_ROTATIONS * GEAR_RATIO);

    public static final Angle TOLERANCE = Degrees.of(5.0);

    public static final Mass CG_MASS = Pounds.of(1);
    public static final Distance CG_DISTANCE = Inches.of(4);

    // lower this value to shift the shot to the right
    public static Angle ADJUSTMENT_TO_ZERO_ANGLE = Degrees.of(-68);

    public static final Distance RADIUS = Inches.of(5);

    // public static final MomentOfInertia MOMENT_OF_INERTIA = UniPtsMath.CalcMomentOfInertia(CG_MASS, CG_DISTANCE);
    public static final MomentOfInertia MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.005);
    public static final Pose3d ORIGIN_POSE3D = new Pose3d(
            Inches.of(-12.0),
            Inches.of(0),
            Inches.of(12),
            new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

    // from arm subsystem
    private static ServoMotorConfigFX getServoConfig()
    {
        ServoMotorConfigFX config = new ServoMotorConfigFX(NAME, TALON_CAN_ID)
                // .withCANcoder(CANCODER_ID, getCANCoderConfig(), GEAR_RATIO, 1.0)
                .withGearRatio(GEAR_RATIO);
        // .withSoftLimits(MIN_ANGLE, MAX_ANGLE);

        config.motorConfig.Feedback.FeedbackRotorOffset = 0.0;

        config.motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        config.motorConfig.CurrentLimits.StatorCurrentLimit = 50;
        config.motorConfig.CurrentLimits.SupplyCurrentLimit = 35;
        config.motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 25;

        // previous kP = 200.0;
        config.motorConfig.Slot0.kP = 225;
        config.motorConfig.Slot0.kS = 1.0;
        config.motorConfig.Slot0.kV = 2.0;
        config.motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.motorConfig.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(2));
        config.motorConfig.MotionMagic.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(15));

        // Fix the orientation
        config.motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.momentOfInertia = MOMENT_OF_INERTIA;

        config.signalTypes = StatusSignalTypes.CurrentsAndPosition;

        return config;
    }

    protected static ServoMotorTalonFX getServoMotor()
    {
        if (RobotBase.isSimulation())
        {
            return new ServoMotorTalonFXSim(getServoConfig());
        }
        else
        {
            return new ServoMotorTalonFX(getServoConfig());
        }
    }

}
