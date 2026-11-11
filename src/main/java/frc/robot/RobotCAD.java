package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

public class RobotCAD
{
    private final RobotState robotState;

    public RobotCAD(RobotState robotState)
    {
        this.robotState = robotState;

    }

    private Rotation2d turretAngle = Rotation2d.kZero; // Robot-relative
    private Rotation2d hoodAngle = Rotation2d.kZero; // Relative to horizontal
    private Rotation2d intakeAngle = Rotation2d.kZero; // Relative to horizontal
    private Rotation2d climberAngle = Rotation2d.kZero; // Relative to horizontal

    public static Transform3d robotToTurret = new Transform3d(
            -0.2,
            -0.14,
            0.34,
            Rotation3d.kZero);

    public static Transform3d turretToHood = new Transform3d(
            0.097,
            0.0,
            0.109,
            Rotation3d.kZero);

    public static Transform3d robotToIntake = new Transform3d(
            0.330200,
            0.0,
            0.222250,
            Rotation3d.kZero);

    public static Transform3d robotToClimber = new Transform3d(
            -0.310873,
            0.0,
            0.454035,
            Rotation3d.kZero);

    public void update()
    {
        // Update angles from subsystems, accounting for original CAD angles
        turretAngle = toRotation2d(robotState.turret.getTurretAngle().unaryMinus().minus(Degrees.of(125)));
        hoodAngle = toRotation2d(robotState.hood.getHoodAngle().minus(Degrees.of(10)));
        // TODO: update intake and climber angles
        intakeAngle = toRotation2d(robotState.intake.getArmAngle().plus(Degrees.of(180)).unaryMinus());
        climberAngle = toRotation2d(robotState.climber.getClimberAngle().plus(Degrees.of(50))); // TODO what angle do we consider horizontal? Degrees.of(0)

        // turret pose needs to transform it from robot origin to turret origin,
        // then rotate by turret angle
        var turretPose = toPose3d(robotToTurret)
                .transformBy(new Transform3d(Translation3d.kZero,
                        new Rotation3d(0.0, 0.0, turretAngle.getRadians())));

        // hood pose needs to transform from turret origin to hood origin,
        // then rotate by hood angle
        var hoodPose = turretPose
                .transformBy(new Transform3d(turretToHood.getX(), turretToHood.getY(), turretToHood.getZ(),
                        new Rotation3d(
                                0.0, hoodAngle.getRadians(), 0.0)));

        // intake pose needs to transform from robot origin to intake origin,
        // then rotate by intake angle
        var intakePose = toPose3d(robotToIntake)
                .transformBy(new Transform3d(Translation3d.kZero,
                        new Rotation3d(0.0, intakeAngle.getRadians(), 0.0)));

        // climber pose needs to transform from robot origin to climber origin,
        // then rotate by climber angle
        var climberPose = toPose3d(robotToClimber)
                .transformBy(new Transform3d(Translation3d.kZero,
                        new Rotation3d(0.0, climberAngle.getRadians(), 0.0)));

        // Logger.recordOutput("CAD/ComponentPoses", new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d());
        Logger.recordOutput("CAD/ComponentPoses", turretPose, hoodPose, intakePose, climberPose);

    }

    public static Pose3d toPose3d(Transform3d transform)
    {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    public static Rotation2d toRotation2d(Angle angle)
    {
        return new Rotation2d(angle.in(Radians));
    }
}
