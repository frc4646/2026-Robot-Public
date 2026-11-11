package frc.robot.configuration;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.CANDeviceId;
import frc.robot.utils.ShootOnTheFly.ShooterLookupTable;

public final class Constants
{
    public static int CAN_TIMEOUT = 100;
    public static boolean IS_REPLAY;
    public static final double LOOP_TIME = 0.02;

    public static final String VISION_ORIN_IP = "10.46.46.102";
    public static final String VISION_POSE_TOPIC = "fused_pose";
    public static final String VISION_SEESTAG_TOPIC = "cam0_has_detections";

    public static final Pose2d TURRET_OFFSET = new Pose2d(Inches.of(-7.75).in(Meters), Inches.of(5.5).in(Meters), new Rotation2d());
    public static final Pose2d INTAKE_OFFSET = new Pose2d(Inches.of(26.5).in(Meters), Inches.of(0).in(Meters), new Rotation2d());
    public static final Distance TRENCH_DUCK_TOLERANCE = Meters.of(1.0);
    public static final double HOOD_DUCK_TIME = 0.25;

    public static final double HUB_PROCESSING_TIME = 0.5; // low end 38 frames, a bit over 0.5 so keeping it safe
    public static final double HUB_AFTER_SHIFT_PROCESS_TIME = 3.0; // from manual

    public static final class SUBSYSTEM_ENABLED
    {
        public static final boolean ARM = false;
    }

    // CAN Device IDs and Busses
    public static final class CAN
    {
        public static final class BUS
        {
            public static final CANBus RIO = CANBus.roboRIO();
            public static final CANBus CANIVORE = new CANBus("canivore");
        }

        public static final CANDeviceId INTAKE_ROLLERS = new CANDeviceId(34, CAN.BUS.CANIVORE);
        public static final CANDeviceId INTAKE_ROLLERS_FOLLOWER = new CANDeviceId(35, CAN.BUS.CANIVORE);

        public static final CANDeviceId INTAKE = new CANDeviceId(36, CAN.BUS.CANIVORE);
        public static final CANDeviceId INTAKE_FOLLOWER = new CANDeviceId(37, CAN.BUS.CANIVORE);
        public static final CANDeviceId INTAKE_CANCODER = new CANDeviceId(38, CAN.BUS.CANIVORE);

        public static final CANDeviceId TURRET = new CANDeviceId(46, CAN.BUS.CANIVORE);
        public static final CANDeviceId TURRET_CANCODER = new CANDeviceId(47, CAN.BUS.CANIVORE);

        public static final CANDeviceId FLYWHEEL = new CANDeviceId(48, CAN.BUS.CANIVORE);
        public static final CANDeviceId FLYWHEEL_FOLLOWER = new CANDeviceId(49, CAN.BUS.CANIVORE);

        public static final CANDeviceId TURRETHOOD = new CANDeviceId(55, CAN.BUS.CANIVORE);

        public static final CANDeviceId CANDI = new CANDeviceId(2, CAN.BUS.CANIVORE);

        public static final CANDeviceId SPINDEXER = new CANDeviceId(39, CAN.BUS.RIO);
        public static final CANDeviceId BELT_WALL = new CANDeviceId(44, CAN.BUS.RIO);
        public static final CANDeviceId BELT_WALL_FOLLOWER = new CANDeviceId(59, CAN.BUS.RIO);
        public static final CANDeviceId FEEDER = new CANDeviceId(45, CAN.BUS.RIO);
        public static final CANDeviceId FEEDER_CAN_RANGE = new CANDeviceId(54, CAN.BUS.RIO);
    }

    public static final class SHOOT_ON_THE_FLY
    {
        public static final double FPS_TO_RPM_RATIO = 100;
        public static final double ROBOT_VECTOR_RPM_FACTOR = 0.6; // TODO: why is this value needed?
        public static final double ROBOT_VECTOR_ROTATION_FACTOR = 0.8; // TODO: why is this value needed?
        public static final double LATENCY_COMPENSATION = 0.1; // TODO: figure out what this should be
    }

    // Swerve Limits and PID gains
    public static final class SWERVE
    {
        public static final double MaxAngularRateAccel = 13;
        public static final double MaxSpeedAccel = 10;
        public static final double MaxAngularRate = 5;
        public static final double MaxSpeed = 5; // OR
        // public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
        public static final double PID_ROTATION_KP = 4;
        public static final double PID_ROTATION_KI = 0;
        public static final double PID_ROTATION_KD = 0;
        public static final double PID_TRANSLATION_KP = 3;
        public static final double PID_TRANSLATION_KI = 0;
        public static final double PID_TRANSLATION_KD = 0;
        public static final AngularVelocity ROTATIONAL_RATE_DEADBAND = DegreesPerSecond.of(2);

        // TODO: update
        public static final Distance ROBOT_AND_BUMPERS_WIDTH = Inches.of(36.0);
        public static final Distance ROBOT_AND_BUMPERS_LENGTH = Inches.of(35.0);

        public static final double MAX_SHOOTING_SWERVE_SPEED = 3.0; // mps?
    }

    public static final class FIELD
    {
    }

    public static final ShooterLookupTable SHOOTER_MAP;
    static
    {
        SHOOTER_MAP = new ShooterLookupTable();
        // TODO: measure time of flights
        SHOOTER_MAP.put(Meters.of(0.0), new ShooterLookupTable.ShooterParameters(RPM.of(1900), Degrees.of(70), 0.7)); // TOF guess
        SHOOTER_MAP.put(Meters.of(1.1), new ShooterLookupTable.ShooterParameters(RPM.of(1900), Degrees.of(70), 0.8)); // TOF guess
        SHOOTER_MAP.put(Meters.of(2.1), new ShooterLookupTable.ShooterParameters(RPM.of(2100), Degrees.of(65), 0.91)); // measured
        SHOOTER_MAP.put(Meters.of(3.051), new ShooterLookupTable.ShooterParameters(RPM.of(2250), Degrees.of(59), 1.0)); // measured
        SHOOTER_MAP.put(Meters.of(4.045), new ShooterLookupTable.ShooterParameters(RPM.of(2400), Degrees.of(54.5), 1.15)); // TOF estimated based on shot at 4.6
        SHOOTER_MAP.put(Meters.of(5.3), new ShooterLookupTable.ShooterParameters(RPM.of(2600), Degrees.of(52.5), 1.4)); // TOF estimated based on shot at 4.6
        SHOOTER_MAP.put(Meters.of(6.3), new ShooterLookupTable.ShooterParameters(RPM.of(2750), Degrees.of(52.5), 1.7)); // TOF guess
    }
    public static final ShooterLookupTable FEED_MAP;
    static
    {
        FEED_MAP = new ShooterLookupTable();
        // TODO: measure time of flights
        FEED_MAP.put(Meters.of(0.0), new ShooterLookupTable.ShooterParameters(RPM.of(2000), Degrees.of(70), 0.7)); // TOF's copied from above
        FEED_MAP.put(Meters.of(1.1), new ShooterLookupTable.ShooterParameters(RPM.of(2000), Degrees.of(70), 0.8));
        FEED_MAP.put(Meters.of(2.1), new ShooterLookupTable.ShooterParameters(RPM.of(2100), Degrees.of(63), 0.91));
        FEED_MAP.put(Meters.of(3.051), new ShooterLookupTable.ShooterParameters(RPM.of(2300), Degrees.of(57), 1.0));
        FEED_MAP.put(Meters.of(4.045), new ShooterLookupTable.ShooterParameters(RPM.of(2500), Degrees.of(53), 1.15));
        FEED_MAP.put(Meters.of(5.3), new ShooterLookupTable.ShooterParameters(RPM.of(2800), Degrees.of(51), 1.4));
        FEED_MAP.put(Meters.of(6.3), new ShooterLookupTable.ShooterParameters(RPM.of(2800), Degrees.of(49), 1.7));
        FEED_MAP.put(Meters.of(7.3), new ShooterLookupTable.ShooterParameters(RPM.of(2900), Degrees.of(45.5), 1.75));
        FEED_MAP.put(Meters.of(8.3), new ShooterLookupTable.ShooterParameters(RPM.of(3000), Degrees.of(45.5), 1.78)); // measured
        FEED_MAP.put(Meters.of(9.3), new ShooterLookupTable.ShooterParameters(RPM.of(3100), Degrees.of(45.5), 1.8));
        FEED_MAP.put(Meters.of(13.0), new ShooterLookupTable.ShooterParameters(RPM.of(4000), Degrees.of(45.5), 1.8));

        // TODO: add points in opponent zone
    }

}
