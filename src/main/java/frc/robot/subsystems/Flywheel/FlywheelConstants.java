package frc.robot.subsystems.Flywheel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.AngularVelocity;
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

public final class FlywheelConstants
{
    public static final String NAME = "Flywheel";

    public static final CANDeviceId TALON_CAN_ID = Constants.CAN.FLYWHEEL;
    // public static final CANDeviceId CANCODER_CAN_ID = Constants.CAN.ARM_CANCODER;

    public static final AngularVelocity ROTATIONS_PER_MINUTE = RPM.of(0);
    public static final AngularVelocity MAX_RPM = RPM.of(5000); // Could be 6000, but changing for testing
    public static final AngularVelocity MIN_RPM = MAX_RPM.times(-1);
    public static final AngularVelocity MIN_SHOOTING_RPM = RPM.of(0);

    public static final AngularVelocity TOLERANCE = RPM.of(500);

    /**
     * Motor shaft to sensor gear ratio. > 1 if sensor is slower than the motor.
     */
    public static final double GEAR_RATIO = 1;

    public static final Distance ROLLER_RADIUS = Inches.of(1.6);

    // CAD measured center of mass
    public static final Mass CG_MASS = Pound.of(2.0);
    public static final Distance CG_DISTANCE = Inches.of(6.0);
    public static final MomentOfInertia MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.0011705586);
    public static final CANDeviceId FOLLOWER_ID = Constants.CAN.FLYWHEEL_FOLLOWER;

    public static final Pose3d ORIGIN = new Pose3d(
            Inches.of(-12.0),
            Inches.of(0.0),
            Inches.of(12),
            new Rotation3d(Degrees.zero(), Degrees.of(90.0), Degrees.zero()));

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
                // .withCANcoder(CANCODER_CAN_ID, getCANCoderConfig(), GEAR_RATIO, 1.0)
                .withGearRatio(GEAR_RATIO)
                .withFollower(FOLLOWER_ID, InvertedValue.CounterClockwise_Positive);

        config.motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        config.motorConfig.CurrentLimits.StatorCurrentLimit = 140;
        config.motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.motorConfig.CurrentLimits.SupplyCurrentLimit = 55;
        config.motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 55;

        config.motorConfig.Slot0.kP = 12.0;
        config.motorConfig.Slot0.kV = 1.0;
        config.motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Fix the orientation
        config.motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.momentOfInertia = MOMENT_OF_INERTIA;

        config.signalTypes = StatusSignalTypes.CurrentsAndVelocity;

        return config;
    }
}
