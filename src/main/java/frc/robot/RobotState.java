package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.FieldUtil;
import frc.robot.configuration.Constants;
import frc.robot.subsystems.Flywheel.FlywheelConstants;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.TurretHood.TurretHoodConstants;
import frc.robot.utils.FieldConstants_2026;
import frc.robot.utils.FieldUtil_2026;
import frc.robot.utils.ShootOnTheFly.ShooterController;

public class RobotState
{

    public final static double LOOKBACK_TIME = 1.0;
    public final static double TIME_OF_FLIGHT_TIME = 1.5;
    // quick fix for line 47 of TurretSubsystem

    public Swerve swerve;
    public Flywheel flywheel;
    public TurretHood hood;
    public Turret turret;
    public Intake intake;
    public Climber climber;
    public Indexer indexer;

    private boolean wonAutonomous;
    // private boolean isLidClosed;
    private double matchTime;
    public boolean visionSeesTag;

    public double timeSinceLastShot;

    public RobotState()
    {
        wonAutonomous = false;
        visionSeesTag = false;
        swerve = new Swerve();
        flywheel = new Flywheel();
        hood = new TurretHood();
        turret = new Turret();
        intake = new Intake();
        climber = new Climber();
        timeSinceLastShot = 1000;
        indexer = new Indexer();
    }

    public class Swerve
    {
        private Pose2d robotPose2d;
        private double speed;
        private boolean shooting;

        private Rotation2d lastRotation;

        Swerve()
        {
            robotPose2d = new Pose2d();
            speed = 0;
            shooting = false;
            lastRotation = new Rotation2d();
        }

        public Pose2d getRobotPose()
        {
            return robotPose2d;
        }

        public void setShooting(boolean shooting)
        {
            this.shooting = shooting;
        }

        public void setLastRotation(Rotation2d lastRotation)
        {
            this.lastRotation = lastRotation;
        }

        public Rotation2d getLastRotation()
        {

            return lastRotation;
        }

        public boolean getShooting()
        {
            return shooting;
        }

        public double getSpeed()
        {
            return speed;
        }

        public void setSpeed(double speed)
        {
            this.speed = speed;
        }

        public boolean isFlat()
        {
            Angle xRot = RobotContainer.SWERVE.getRotation3d().getMeasureX();
            Angle yRot = RobotContainer.SWERVE.getRotation3d().getMeasureY();
            if (xRot.isNear(Degrees.of(0), Degrees.of(0.5))
                    && yRot.isNear(Degrees.of(0), Degrees.of(0.5)))
            {
                return true;
            }
            return false;
        }

        public void setRobotPose(Pose2d robotPose2d)
        {
            this.robotPose2d = robotPose2d;
        }
    }

    public boolean canShoot()
    {
        // Conditions: Flywheel up to speed, in alliance zone, turret hood at correct angle
        /*
         * Flywheel: actual - target <= tolerance
         * Hood: actual - target <= tolerance
         * In alliance zone true
         * Function:
         * return true if robot should shoot hub or stockpile
         */

        boolean canShoot = (RPM.of(flywheel.getError().abs(RPM)).lte(FlywheelConstants.TOLERANCE))
                && (flywheel.velo.gte(Units.RPM.of(1000.0)))
                && Degrees.of(hood.getError().abs(Degrees)).lte(TurretHoodConstants.POSITION_TOLERANCE)
                && Degrees.of(turret.getError().abs(Degrees)).lte(TurretConstants.TOLERANCE)
                && (!turret.getWrapping())
                && safeToLiftHood()
                && !FieldUtil_2026.isBehindHub(this);
        // && !(FieldUtil_2026.isInAllianceZone(this) && !isActiveShift()); // trying to shoot in the hub outside of ouractive
        // shift TODO: include time o
        // flight instead of a default of 1.5
        // && !FieldUtil_2026.isBehindTower(this);
        return canShoot;

        /*
         * ! in AZ || (in Az && time check)
         * FieldUtil_2026.isInAllianceZone(this))
         * isActiveShift()
         */

    }

    public class Flywheel
    {
        AngularVelocity velo;
        AngularVelocity shootingVelo;
        AngularVelocity error;

        public Flywheel()
        {
            velo = RPM.of(5000);
            error = RPM.of(1000);
            shootingVelo = RPM.of(5000);
        }

        public void setVelo(AngularVelocity velo)
        {
            this.velo = velo;
        }

        public void setShootingVelo(AngularVelocity velo)
        {
            this.shootingVelo = velo;
        }

        public void setError(AngularVelocity error)
        {
            this.error = error;
        }

        public AngularVelocity getError()
        {
            return error;
        }

        public AngularVelocity getAngularVelocity()
        {
            return velo;
        }

        public AngularVelocity getShootingVelocity()
        {
            return shootingVelo;
        }
    }

    public class Indexer
    {
        private double rollerSpeed;
        private boolean isLidClosed;

        Indexer()
        {
            rollerSpeed = 0;
        }

        public void setRollerSpeed(double rollerSpeed)
        {
            this.rollerSpeed = rollerSpeed;
        }

        public double getRollerSpeed()
        {
            return rollerSpeed;
        }

        public void setIsLidClosed(boolean isLidClosed)
        {
            this.isLidClosed = isLidClosed;
        }

        public boolean getIsLidClosed()
        {
            return isLidClosed;
        }
    }

    public class Intake
    {
        private double rollerSpeed;
        private Angle intakeArmAngle;

        Intake()
        {
            rollerSpeed = 0;
            intakeArmAngle = Degrees.of(0);
        }

        public void setRollerSpeed(double rollerSpeed)
        {
            this.rollerSpeed = rollerSpeed;
        }

        public double getRollerSpeed()
        {
            return rollerSpeed;
        }

        public void setArmAngle(Angle intakeArmAngle)
        {
            this.intakeArmAngle = intakeArmAngle;
        }

        public Angle getArmAngle()
        {
            return intakeArmAngle;
        }
    }

    public class Climber
    {
        private Angle climberAngle;

        Climber()
        {
            climberAngle = Degrees.of(0);
        }

        public void setClimberAngle(Angle climberAngle)
        {
            this.climberAngle = climberAngle;
        }

        public Angle getClimberAngle()
        {
            return climberAngle;
        }
    }

    public class TurretHood
    {

        private Angle turretHoodAngle;
        private Angle error;

        TurretHood()
        {
            turretHoodAngle = Degrees.of(0);
        }

        public void setHoodAngle(Angle intakeHoodAngle)
        {
            this.turretHoodAngle = intakeHoodAngle;
        }

        public void setError(double error)
        {
            this.error = Degrees.of(error);
        }

        public Angle getError()
        {
            return error;
        }

        public Angle getHoodAngle()
        {
            return turretHoodAngle;
        }
    }

    public class Turret // TODO: implement turret
    {
        private Angle turretAngle;
        private Angle error;
        private boolean wrapping;

        Turret()
        {
            turretAngle = Degrees.of(0);
            error = Degrees.of(999);
            wrapping = false;
        }

        public void setError(double error)
        {
            this.error = Degrees.of(error);
        }

        public Angle getError()
        {
            return error;
        }

        public void setTurretAngle(Angle turretAngle)
        {
            this.turretAngle = turretAngle;
        }

        public Angle getTurretAngle()
        {
            return turretAngle;
        }

        public void setWrapping(boolean wrapping)
        {
            this.wrapping = wrapping;
        }

        public boolean getWrapping()
        {
            return wrapping;
        }
    }

    public boolean safeToLiftHood()
    {
        double xPosition = getTurretPose().getX();
        double xVelo = RobotContainer.SWERVE.getState().Speeds.vxMetersPerSecond;
        return !pointInTrenchZone(xPosition) &&
                !goingToHitTrench(xPosition, xVelo, Constants.HOOD_DUCK_TIME);
    }

    public boolean safeToLiftHopperHood()
    {
        double xPosition = getIntakeCenterPose().getX();
        double yPosition = getIntakeCenterPose().getY();
        double xVelo = RobotContainer.SWERVE.getState().Speeds.vxMetersPerSecond;
        return !(((yPosition > FieldConstants_2026.LinesHorizontal.leftBumpStart)
                ||
                (yPosition < FieldConstants_2026.LinesHorizontal.rightBumpEnd))) ||

        // X values
                ((xPosition < FieldConstants_2026.LinesVertical.allianceZone) ||
                        ((xPosition > FieldConstants_2026.LinesVertical.neutralZoneNear) &&
                        // Opposing side v
                                (xPosition < FieldConstants_2026.LinesVertical.neutralZoneFar))
                        ||
                        (xPosition > FieldConstants_2026.LinesVertical.oppAllianceZone));
    }

    public boolean goingToHitTrench(double x, double xVelo, double secondsToMoveHood)
    {
        double trenchWiggleRoom = Constants.TRENCH_DUCK_TOLERANCE.in(Meters);

        return (xVelo > 0 &&
                (x < FieldConstants_2026.LinesVertical.hubCenter - trenchWiggleRoom) &&
                (x + xVelo * secondsToMoveHood > FieldConstants_2026.LinesVertical.hubCenter - trenchWiggleRoom))
                || (xVelo < 0 &&
                        (x > FieldConstants_2026.LinesVertical.hubCenter + trenchWiggleRoom) &&
                        (x + xVelo * secondsToMoveHood < FieldConstants_2026.LinesVertical.hubCenter + trenchWiggleRoom))
                ||
                (xVelo > 0 &&
                        (x < FieldConstants_2026.LinesVertical.oppHubCenter - trenchWiggleRoom) &&
                        (x + xVelo * secondsToMoveHood > FieldConstants_2026.LinesVertical.oppHubCenter - trenchWiggleRoom))
                || (xVelo < 0 &&
                        (x > FieldConstants_2026.LinesVertical.oppHubCenter + trenchWiggleRoom) &&
                        (x + xVelo * secondsToMoveHood < FieldConstants_2026.LinesVertical.oppHubCenter + trenchWiggleRoom));
    }

    public boolean pointInTrenchZone(double x)
    {
        double trenchWiggleRoom = Constants.TRENCH_DUCK_TOLERANCE.in(Meters);
        return ((x > FieldConstants_2026.LinesVertical.hubCenter - trenchWiggleRoom &&
                x < FieldConstants_2026.LinesVertical.hubCenter + trenchWiggleRoom) ||
                (x > FieldConstants_2026.LinesVertical.oppHubCenter - trenchWiggleRoom &&
                        x < FieldConstants_2026.LinesVertical.oppHubCenter + trenchWiggleRoom));
    }

    public void update()
    {
        String gameData;
        gameData = DriverStation.getGameSpecificMessage();
        if (gameData.length() > 0)
        {
            boolean redWonAutonomous = false;
            switch (gameData.charAt(0))
            {
                case 'B':
                    redWonAutonomous = false;
                    // Blue case code
                    break;
                case 'R':
                    redWonAutonomous = true;
                    // Red case code
                    break;
                default:
                    // This is corrupt data
                    break;
            }
            // this works as (red alliance and red won) or (blue alliance and blue won)
            wonAutonomous = RobotContainer.isOnRedAlliance() == redWonAutonomous;
        }
        Logger.recordOutput("robotState/timeSinceLastShot", timeSinceLastShot);
        // Logger.recordOutput("robotState/canShoot", canShoot());
        // Logger.recordOutput("robotState/flywheelTol",
        // (flywheel.getError().lte(FlywheelConstants.TOLERANCE)));
        // Logger.recordOutput("robotState/flyRPM", (flywheel.velo.gte(Units.RPM.of(1000.0))));
        // Logger.recordOutput("robotState/rsHoodAngle", (hood.turretHoodAngle));
        // Logger.recordOutput("robotState/hood?", (hood.getError().lte(TurretHoodConstants.POSITION_TOLERANCE)));
        // Logger.recordOutput("robotState/turret?", (turret.getError().lte(TurretConstants.TOLERANCE)));
        // Logger.recordOutput("robotState/swerveVector", calculateSwerveVector());
        // Logger.recordOutput("robotState/hoodSafe", safeToLiftHood());
        // Logger.recordOutput("robotState/turretPose", getTurretPose());
        // Logger.recordOutput("robotState/inAllianceZone", FieldUtil_2026.isInAllianceZone(this));
        // Logger.recordOutput("robotState/preSOTF/rpm", flywheel.getAngularVelocity());
        // Logger.recordOutput("robotState/preSOTF/angle", hood.getHoodAngle());
        // Logger.recordOutput("robotState/ChoosenTarget", FieldUtil.flipFieldPointIfRed(FieldUtil_2026.chooseTarget(this)));
        Logger.recordOutput("CanShoot", canShoot());
        Logger.recordOutput("isBehindHub", FieldUtil_2026.isBehindHub(this));

        Logger.recordOutput("robotState/ActiveShift", isActiveShift());
        Logger.recordOutput("robotState/matchCountDown", getMatchCountDown());

        // ShotTrajectoryVisualizer.logTrajectory(
        // "ShotAdjusted",
        // new Translation3d(swerve.getRobotPose().getX(), swerve.getRobotPose().getY(), 0.5), // launch position
        // getAdjustedRotationToTarget(FieldConstants_2026.Hub.HubPose).getRadians(), // field-relative yaw
        // hood.getHoodAngle().in(Radians), // pitch (45 degrees TODO)
        // FeetPerSecond.of(calculateAdjustedFPS(FieldConstants_2026.Hub.HubPose)).in(MetersPerSecond) // velocity
        // );
        // ShotTrajectoryVisualizer.logTrajectory(
        // "ShotUnadjusted",
        // new Translation3d(swerve.getRobotPose().getX(), swerve.getRobotPose().getY(), 0.5), // launch position
        // calculateBallVector(FieldConstants_2026.Hub.HubPose).getAngle().getRadians(), // field-relative yaw
        // hood.getHoodAngle().in(Radians), // pitch (45 degrees TODO)
        // FeetPerSecond.of(calculateFPSFromRPM()).in(MetersPerSecond) // velocity
        // );

        matchTime = Timer.getMatchTime(); // TODO: docs say this is not guaranteed to work 100%, check if it's fine
    }

    public Pose2d getTurretPose()
    {
        return swerve.getRobotPose().plus(new Transform2d(
                Constants.TURRET_OFFSET.getX(),
                Constants.TURRET_OFFSET.getY(),
                new Rotation2d(Degrees.of(0))));
    }

    public Pose2d getIntakeCenterPose()
    {
        return swerve.getRobotPose().plus(new Transform2d(
                Constants.INTAKE_OFFSET.getX(),
                Constants.INTAKE_OFFSET.getY(),
                new Rotation2d(Degrees.of(0))));
    }

    public double getMatchCountDown()
    {
        return matchTime;
    }

    public boolean wonAutonomous()
    {
        return wonAutonomous;
    }

    public boolean isActiveShift(double timeRemaining, double timeOfFlight)
    {

        /*
         * < 30: both active
         * 30 - 55: shift 4 (Auto W)
         * 55 - 80: shift 3 (Auto L)
         * 80 - 105: shift 2 (Auto W)
         * 105 - 130: shift 1 (Auto L)
         * > 130: both active
         */
        // TODO: expand the bounds of each shift, to work with shooting/processing time at the start, and the extra 3 seconds at
        // the end.

        double shotToScoredTime = Constants.HUB_PROCESSING_TIME + timeOfFlight;
        if ((timeRemaining > 130 - Constants.HUB_AFTER_SHIFT_PROCESS_TIME + shotToScoredTime)
                || timeRemaining < 30 + shotToScoredTime) // always active
        {

            return true;
        }
        if (wonAutonomous())
        {
            return (timeRemaining <= 55 + shotToScoredTime)
                    || (timeRemaining > (80 - Constants.HUB_AFTER_SHIFT_PROCESS_TIME + (shotToScoredTime))
                            && (timeRemaining <= 105 + shotToScoredTime));
        }
        else
        {
            return (timeRemaining > (55 - Constants.HUB_AFTER_SHIFT_PROCESS_TIME + (shotToScoredTime))
                    && (timeRemaining <= 80 + shotToScoredTime))
                    || (timeRemaining > (105 - Constants.HUB_AFTER_SHIFT_PROCESS_TIME + (shotToScoredTime))
                            && (timeRemaining <= 130 + shotToScoredTime));
        }
    }

    public boolean isActiveShift()
    {
        return isActiveShift(getMatchCountDown(), TIME_OF_FLIGHT_TIME);
    }

    public boolean isActiveShift(double timeOfFlight)
    {
        return isActiveShift(getMatchCountDown(), timeOfFlight);
    }

    public double getTimeUntilActiveShift()
    {
        return getTimeUntilActiveShift(TIME_OF_FLIGHT_TIME);
    }

    public double getTimeUntilActiveShift(double timeOfFlight)
    {
        if (isActiveShift(timeOfFlight))
        {
            return 0.0;
        }

        double timeRemaining = getMatchCountDown();
        double shotToScoredTime = Constants.HUB_PROCESSING_TIME + timeOfFlight;
        boolean wonAuto = wonAutonomous();

        // Shift timing breakdown:
        // > 140: autonomous (both active)
        // 130 - 140: transition (both active)
        // 105 - 130: shift 1 (Auto L)
        // 80 - 105: shift 2 (Auto W)
        // 55 - 80: shift 3 (Auto L)
        // 30 - 55: shift 4 (Auto W)
        // < 30: both active

        // Active begins when timeRemaining drops to (raw shift upper bound) + shotToScoredTime,
        // matching the upper bounds used by isActiveShift.
        if (wonAuto)
        {
            // Won auto: active at >130, 80-105, 30-55, <30
            // Inactive at: 105-130, 55-80
            if (timeRemaining > 105 + shotToScoredTime)
            {
                return timeRemaining - (105 + shotToScoredTime);
            }
            else // 55-80
            {
                return timeRemaining - (55 + shotToScoredTime);
            }
        }
        else
        {
            // Lost auto: active at >130, 105-130, 55-80, <30
            // Inactive at: 80-105, 30-55
            if (timeRemaining > 80 + shotToScoredTime)
            {
                return timeRemaining - (80 + shotToScoredTime);
            }
            else // 30-55
            {
                return timeRemaining - (30 + shotToScoredTime);
            }
        }
    }

    public double getTimeUntilInactiveShift()
    {
        return getTimeUntilInactiveShift(TIME_OF_FLIGHT_TIME);
    }

    public double getTimeUntilInactiveShift(double timeOfFlight)
    {
        if (!isActiveShift(timeOfFlight))
        {
            return 0.0;
        }

        double timeRemaining = getMatchCountDown();
        double shotToScoredTime = Constants.HUB_PROCESSING_TIME + timeOfFlight;
        double shiftEndBuffer = Constants.HUB_AFTER_SHIFT_PROCESS_TIME - shotToScoredTime;
        boolean wonAuto = wonAutonomous();

        // Shift timing breakdown:
        // > 140: autonomous (both active)
        // 130 - 140: transition (both active)
        // 105 - 130: shift 1 (Auto L)
        // 80 - 105: shift 2 (Auto W)
        // 55 - 80: shift 3 (Auto L)
        // 30 - 55: shift 4 (Auto W)
        // < 30: both active

        // Always active until match ends, no inactive shift coming
        if (timeRemaining <= 30 + shotToScoredTime)
        {
            return 200.0;
        }

        // Both active top-of-match - find next inactive shift
        if (timeRemaining > 130 - shiftEndBuffer)
        {
            // Next inactive: 105-130 for winner, 80-105 for loser
            return wonAuto ? timeRemaining - (130 - shiftEndBuffer)
                    : timeRemaining - (105 - shiftEndBuffer);
        }

        // Active ends when timeRemaining drops to (raw shift lower bound) - shiftEndBuffer,
        // matching the lower bounds used by isActiveShift.
        if (wonAuto)
        {
            // Won auto: active at 80-105, 30-55
            // Next inactive at: 55-80 after 80-105, or <30 (none) after 30-55
            if (timeRemaining > 80 - shiftEndBuffer && timeRemaining <= 105 + shotToScoredTime)
            {
                return timeRemaining - (80 - shiftEndBuffer);
            }
            else // 30-55, no more inactive shifts
            {
                return 200.0;
            }
        }
        else
        {
            // Lost auto: active at 105-130, 55-80
            // Next inactive at: 80-105 after 105-130, or 30-55 after 55-80
            if (timeRemaining > 105 - shiftEndBuffer && timeRemaining <= 130 + shotToScoredTime)
            {
                return timeRemaining - (105 - shiftEndBuffer);
            }
            else // 55-80
            {
                return timeRemaining - (55 - shiftEndBuffer);
            }
        }
    }

    // ON THE FLY CALCULATION (steps in order)

    public Translation2d calculateTurretTranslationVector()
    {
        var speeds = RobotContainer.SWERVE.getState().Speeds;
        var robotAngle = RobotContainer.SWERVE.getState().Pose.getRotation();
        Translation2d robotSpaceVector = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        // Get turret offset in robot space
        Translation2d turretOffset = new Translation2d(Constants.TURRET_OFFSET.getX(), Constants.TURRET_OFFSET.getY());

        // Calculate velocity imparted by robot rotation: v = ω × r
        // The velocity is perpendicular to the offset vector
        double angularVelocity = speeds.omegaRadiansPerSecond;
        double rotationVelocityMagnitude = angularVelocity * turretOffset.getNorm();

        // The direction perpendicular to the offset (rotated 90 degrees counterclockwise)
        Rotation2d rotationVelocityDirection = turretOffset.getAngle().plus(Rotation2d.fromDegrees(90));
        Translation2d rotationVelocity = new Translation2d(rotationVelocityMagnitude, rotationVelocityDirection);

        // Total velocity in robot space = linear velocity + rotation-induced velocity
        Translation2d totalRobotSpaceVector = robotSpaceVector.plus(rotationVelocity);

        if (totalRobotSpaceVector.getNorm() == 0)
        {
            return new Translation2d();
        }

        Rotation2d FieldSpaceDirection = robotAngle.plus(totalRobotSpaceVector.getAngle());

        return new Translation2d(totalRobotSpaceVector.getNorm(),
                FieldSpaceDirection);
    }

    public ShooterController.ShotValues calculateShotValues(Pose2d target, double latencyCompensation)
    {
        return ShooterController.calculate(
                getTurretPose().getTranslation(),
                calculateTurretTranslationVector(),
                FieldUtil.flipFieldPointIfRed(target).getTranslation(),
                latencyCompensation);
    }

}
