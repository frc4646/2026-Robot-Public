package frc.lib.servomotor;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.configuration.Constants;

public class ServoMotorViz
{
    // All lengths are in Meters

    private static final LoggedMechanism2d robot = new LoggedMechanism2d(
            Constants.SWERVE.ROBOT_AND_BUMPERS_LENGTH.in(Meters),
            Feet.of(7).in(Meters), // 7 feet tall
            new Color8Bit("#222222"));

    static
    {
        Distance bumperHeight = Inches.of(3.25);
        Distance bumperThickness = Inches.of(3.5);
        Color8Bit bumperColor = new Color8Bit("#550000");

        robot.getRoot("BumpersL",
                0.0,
                bumperHeight.in(Meters))
                .append(new LoggedMechanismLigament2d(
                        "BumpersL",
                        bumperThickness.in(Meters),
                        0,
                        13,
                        bumperColor));
        robot.getRoot("BumpersR",
                Constants.SWERVE.ROBOT_AND_BUMPERS_LENGTH.minus(bumperThickness).in(Meters),
                bumperHeight.in(Meters))
                .append(new LoggedMechanismLigament2d(
                        "BumpersR",
                        bumperThickness.in(Meters),
                        0,
                        13,
                        bumperColor));
    }

    private Optional<Supplier<Distance>> distanceSupplier = Optional.empty();
    private Optional<Supplier<Angle>> angleSupplier = Optional.empty();

    private final LoggedMechanismLigament2d mechanism;

    /**
     * Add an elevator mechanism visualization to the robot at the specified mounting pose.
     * 
     * @param name
     *            Name of the mechanism
     * @param mountingPose
     *            Pose of the mechanism's root relative to the robot origin. X is forward, Z is up.
     * @param distanceSupplier
     *            Supplier that provides the current distance of the mechanism
     * @param config
     *            Visualization configuration
     * @return The created ServoMotorViz instance
     */
    public static ServoMotorViz addElevator(String name, Pose3d mountingPose, Supplier<Distance> distanceSupplier, VizConfig config)
    {
        LoggedMechanismRoot2d root = robot.getRoot(
                name + "Root",
                getXOffset(mountingPose.getMeasureX()).in(Meters),
                mountingPose.getMeasureZ().in(Meters));

        ServoMotorViz viz = new ServoMotorViz(name, root, config);

        viz.distanceSupplier = Optional.of(distanceSupplier);
        viz.angleSupplier = Optional.empty();

        return viz;
    }

    /**
     * Add an elevator mechanism visualization to the robot, mounted to an existing mechanism.
     * 
     * @param name
     *            Name of the mechanism
     * @param mountingMechanism
     *            Mechanism to which this mechanism is mounted
     * @param distanceSupplier
     *            Supplier that provides the current distance of the mechanism
     * @param config
     *            Visualization configuration
     * @return The created ServoMotorViz instance
     */
    public static ServoMotorViz addElevator(String name, LoggedMechanismLigament2d mountingMechanism, Supplier<Distance> distanceSupplier,
            VizConfig config)
    {
        ServoMotorViz viz = new ServoMotorViz(name, mountingMechanism, config);

        viz.distanceSupplier = Optional.of(distanceSupplier);
        viz.angleSupplier = Optional.empty();

        return viz;
    }

    /**
     * Add a pivoting (arm) mechanism visualization to the robot at the specified mounting pose.
     * 
     * @param name
     *            Name of the mechanism
     * @param mountingPose
     *            Pose of the mechanism's root relative to the robot origin. X is forward, Z is up.
     * @param angleSupplier
     *            Supplier that provides the current angle of the mechanism
     * @param config
     *            Visualization configuration
     * @return The created ServoMotorViz instance
     */
    public static ServoMotorViz addPivot(String name, Pose3d mountingPose, Supplier<Angle> angleSupplier, VizConfig config)
    {
        LoggedMechanismRoot2d root = robot.getRoot(
                name + "Root",
                getXOffset(mountingPose.getMeasureX()).in(Meters),
                mountingPose.getMeasureZ().in(Meters));

        ServoMotorViz viz = new ServoMotorViz(name, root, config);

        viz.distanceSupplier = Optional.empty();
        viz.angleSupplier = Optional.of(angleSupplier);

        return viz;
    }

    /**
     * Add a pivoting (arm) mechanism visualization to the robot, mounted to an existing mechanism.
     * 
     * @param name
     *            Name of the mechanism
     * @param origin
     *            Mechanism to which this mechanism is mounted
     * @param angleSupplier
     *            Supplier that provides the current angle of the mechanism
     * @param config
     *            Visualization configuration
     * @return The created ServoMotorViz instance
     */
    public static ServoMotorViz addPivot(String name, LoggedMechanismLigament2d origin, Supplier<Angle> angleSupplier, VizConfig config)
    {

        ServoMotorViz viz = new ServoMotorViz(name, origin, config);

        viz.distanceSupplier = Optional.empty();
        viz.angleSupplier = Optional.of(angleSupplier);

        return viz;
    }

    /**
     * Add a pivoting elevator mechanism visualization to the robot at the specified mounting pose.
     * 
     * @param name
     *            Name of the mechanism
     * @param mountingPose
     *            Pose of the mechanism's root relative to the robot origin. X is forward, Z is up.
     * @param angleSupplier
     *            Supplier that provides the current angle of the mechanism
     * @param distanceSupplier
     *            Supplier that provides the current distance of the mechanism
     * @param config
     *            Visualization configuration
     * @return The created ServoMotorViz instance
     */
    public static ServoMotorViz addPivotingElevator(String name, Pose3d mountingPose, Supplier<Angle> angleSupplier, Supplier<Distance> distanceSupplier,
            VizConfig config)
    {
        LoggedMechanismRoot2d root = robot.getRoot(
                name + "Root",
                getXOffset(mountingPose.getMeasureX()).in(Meters),
                mountingPose.getMeasureZ().in(Meters));

        ServoMotorViz viz = new ServoMotorViz(name, root, config);

        viz.distanceSupplier = Optional.of(distanceSupplier);
        viz.angleSupplier = Optional.of(angleSupplier);

        return viz;
    }

    /**
     * Add a pivoting (arm) mechanism visualization to the robot, mounted to an existing mechanism.
     * 
     * @param name
     *            Name of the mechanism
     * @param origin
     *            Mechanism to which this mechanism is mounted
     * @param angleSupplier
     *            Supplier that provides the current angle of the mechanism
     * @param distanceSupplier
     *            Supplier that provides the current distance of the mechanism
     * @param config
     *            Visualization configuration
     * @return The created ServoMotorViz instance
     */
    public static ServoMotorViz addPivotingElevator(String name, LoggedMechanismLigament2d origin, Supplier<Angle> angleSupplier,
            Supplier<Distance> distanceSupplier, VizConfig config)
    {
        ServoMotorViz viz = new ServoMotorViz(name, origin, config);

        viz.distanceSupplier = Optional.of(distanceSupplier);
        viz.angleSupplier = Optional.of(angleSupplier);

        return viz;
    }

    private ServoMotorViz(String name, LoggedMechanismLigament2d root, VizConfig config)
    {
        mechanism = root.append(
                new LoggedMechanismLigament2d(
                        name,
                        config.getDefaultLength().in(Meters),
                        config.getDefaultAngle().in(Degrees),
                        6,
                        new Color8Bit(config.getColor())));
    }

    private ServoMotorViz(String name, LoggedMechanismRoot2d root, VizConfig config)
    {
        mechanism = root.append(
                new LoggedMechanismLigament2d(
                        name,
                        config.getDefaultLength().in(Meters),
                        config.getDefaultAngle().in(Degrees),
                        6,
                        new Color8Bit(config.getColor())));
    }

    /**
     * Log the root mechanism. Call periodically in robot code.
     */
    public static void logRoot()
    {
        Logger.recordOutput("Mechanism2d/Robot", robot);
    }

    /**
     * Get the root mechanism for 3D conversion
     *
     * @return The root LoggedMechanism2d
     */
    public static LoggedMechanism2d getRobotMechanism()
    {
        return robot;
    }

    /**
     * Update the mechanism visualization. Call periodically for each ligament in robot code.
     */
    public void update()
    {
        if (distanceSupplier.isPresent())
        {
            mechanism.setLength(distanceSupplier.get().get().in(Meters));
        }
        if (angleSupplier.isPresent())
        {
            mechanism.setAngle(angleSupplier.get().get().in(Degrees));
        }
    }

    private static Distance getXOffset(Distance mechanismX)
    {
        return mechanismX.plus(Constants.SWERVE.ROBOT_AND_BUMPERS_LENGTH.div(2.0));
    }

    public LoggedMechanismLigament2d getMechanism()
    {
        return mechanism;
    }

    public static class VizConfig
    {
        private final Color color;
        private final Angle defaultAngle;
        private final Distance defaultLength;

        public VizConfig(Color color, Angle defaultAngle, Distance defaultLength)
        {
            this.color = color;
            this.defaultAngle = defaultAngle;
            this.defaultLength = defaultLength;
        }

        public Color getColor()
        {
            return color;
        }

        public Angle getDefaultAngle()
        {
            return defaultAngle;
        }

        public Distance getDefaultLength()
        {
            return defaultLength;
        }

    }
}
