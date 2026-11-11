package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.factories.CANcoderFactory;
import frc.lib.servomotor.ServoMotorSubsystem;
import frc.lib.servomotor.motors.ServoMotorTalonFX;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.configuration.Constants;
import frc.robot.utils.FieldUtil_2026;

public class TurretSubsystem extends ServoMotorSubsystem<ServoMotorTalonFX>
{

    private RobotState robotState;
    private Angle positionCmd;
    private Angle turretSeed;

    private Angle initialDiff;
    private boolean calculatedInitDiff;
    // ServoMotorViz lineOne;

    private final CANcoder cancoder;
    Angle angleToHub;
    Angle previousAngleToHub;
    Angle adjustedAngleToHub;
    Angle extraRotations;

    Boolean wrapping;

    Voltage yawFeedForward;

    public TurretSubsystem(RobotState robotState)
    {
        super(TurretConstants.getServoMotor());

        this.robotState = robotState;

        cancoder = CANcoderFactory.make(TurretConstants.CANCODER_ID, CANcoderFactory.makeDefaultConfig(Degrees.of(0)));

        // force read the sensor signals
        var absolutePositionSignal = ((ServoMotorTalonFX) getServoMotor()).getTalon().getPosition();
        var cancoderPositionSignal = cancoder.getAbsolutePosition();
        BaseStatusSignal.refreshAll(absolutePositionSignal, cancoderPositionSignal);
        turretSeed = solveForTurret(absolutePositionSignal.getValue().times(TurretConstants.GEAR_RATIO), cancoderPositionSignal.getValue());

        // setCurrentPositionAsZero();
        calculatedInitDiff = false;
        yawFeedForward = Volts.of(0);
        wrapping = false;
        // if (turretSeed.gte(Degrees.of(180)))
        // {
        // turretSeed = turretSeed.minus(Degrees.of(360));
        // }
        // set the Kraken to the correct total rotations
        // getServoMotor().setConfigCurrentPosition(turretSeed);

        // set default command to save current position, then hold it
        setDefaultCommand(Commands.runOnce(() ->
        {
            positionCmd = getActuals().getAngle();
        }).andThen(getServoCommands().setTargetAngle(() -> positionCmd))
                .withName(getName() + " Maintain Current (default)"));
        extraRotations = Degrees.of(0);
        previousAngleToHub = Rotations.of(100);
    }

    @Override
    public void periodic()
    {
        super.periodic();
        robotState.turret.setTurretAngle(getActuals().getAngle());

        angleToHub = new Rotation2d(robotState.calculateShotValues(FieldUtil_2026.chooseTarget(robotState),
                Constants.SHOOT_ON_THE_FLY.LATENCY_COMPENSATION).TurretAngle())
                        .plus(new Rotation2d(TurretConstants.ADJUSTMENT_TO_ZERO_ANGLE))
                        .minus(robotState.swerve.getRobotPose().getRotation())
                        .unaryMinus()
                        .getMeasure();

        AngularVelocity robotYawRate = RobotContainer.SWERVE.getGyroRate();
        yawFeedForward = Volts.of(robotYawRate.in(DegreesPerSecond) * TurretConstants.FEED_FORWARD_RATIO);

        wrapping = Math.abs(getErrorAngle().in(Degrees)) > (20);
        Logger.recordOutput(getName() + "/wrapping", wrapping);

        robotState.turret.setWrapping(wrapping);

        // initialize previous angle to be current angle so we don't accidentally wrap at startup
        if (previousAngleToHub.equals(Rotations.of(100)))
        {
            previousAngleToHub = angleToHub;
        }

        adjustedAngleToHub = angleToHub;

        // if we see a big change that means we are wrapping around, add/subtract a full rotation to adjust for the angle
        // wrap
        if (angleToHub.minus(previousAngleToHub).gt(Degrees.of(180)))
        {
            extraRotations = extraRotations.minus(Rotations.of(1));
        }
        if (angleToHub.minus(previousAngleToHub).lt(Degrees.of(-180)))
        {
            extraRotations = extraRotations.plus(Rotations.of(1));
        }
        adjustedAngleToHub = extraRotations.plus(angleToHub);

        // then, if we reach our actual max, manually wrap back around to the correct rotation by adding or subtracting a
        // rotation
        if (adjustedAngleToHub.lt(TurretConstants.MIN_ANGLE))
        {
            extraRotations = extraRotations.plus(Rotations.of(1));
        }
        if (adjustedAngleToHub.gt(TurretConstants.MAX_ANGLE))
        {
            extraRotations = extraRotations.minus(Rotations.of(1));
        }
        adjustedAngleToHub = extraRotations.plus(angleToHub);

        if (!adjustedAngleToHub.equals(Degrees.of(0)))
        {
            Logger.recordOutput(getName() + "/adjustedAngleToHub", adjustedAngleToHub.in(Degrees));
        }

        robotState.turret.setError(getErrorAngle().in(Degrees));
        if (!calculatedInitDiff && !getActuals().getAngle().equals(Degrees.of(0)))
        {
            initialDiff = getActuals().getAngle().minus(turretSeed);
            Logger.recordOutput("Turret/initialDiff", initialDiff);

            calculatedInitDiff = true;
        }
        if (RobotBase.isSimulation() && !calculatedInitDiff)
        {
            initialDiff = Degrees.of(0);

            calculatedInitDiff = true;
        }

        Logger.recordOutput(getName() + "/ActualPositionRotations", getActuals().getAngle().minus(initialDiff));

        // Logger.recordOutput(getName() + "/targetPose",
        // FieldUtil.flipFieldPointIfRed(FieldUtil_2026.chooseTarget(robotState)));
        // update the global robot state, Causes error
        // robotState.turret.setAngle(getActuals().getAngle());

        // save previous angle
        previousAngleToHub = angleToHub;

        if (TurretConstants.DEBUG_POSITIONS)
        {
            var absolutePositionSignal = ((ServoMotorTalonFX) getServoMotor()).getTalon().getPosition();
            var cancoderPositionSignal = cancoder.getAbsolutePosition();
            BaseStatusSignal.refreshAll(absolutePositionSignal, cancoderPositionSignal);
            turretSeed = solveForTurret(absolutePositionSignal.getValue().times(TurretConstants.GEAR_RATIO), cancoderPositionSignal.getValue());
        }

    }

    public Command goToZeroPositionCommand()
    {
        return getServoCommands().setTargetAngle_UntilAtAngle(() -> Degrees.of(0), Degrees.of(3));
    }

    public Command goToPositiveCommand()
    {
        return getServoCommands().setTargetAngle_UntilAtAngle(() -> Degrees.of(10), Degrees.of(5));
    }

    public Command goToNegativeCommand()
    {
        return getServoCommands().setTargetAngle_UntilAtAngle(() -> Degrees.of(-10), Degrees.of(5));
    }

    public Command pointToHubCommand()
    {
        return setAngle(() -> adjustedAngleToHub);
    }

    public Command goTo10()
    {
        return setAngle(() -> Degrees.of(10));
    }

    public Command goTo60()
    {
        return setAngle(() -> Degrees.of(60));
    }

    public Command setAngle(Supplier<Angle> angle)
    {
        return setAngleAdjusted(angle);
    }

    public Command setAngleAdjusted(Supplier<Angle> actualAngle)
    {

        return getServoCommands().setTargetAngle(() ->
        {
            final var motorAngle = new Object()
            {
                Angle angle = actualAngle.get();
            };
            Logger.recordOutput(getName() + "/angleBefore", motorAngle.angle);
            if (motorAngle.angle.gt(TurretConstants.MAX_ANGLE))
                motorAngle.angle = motorAngle.angle.minus(Degrees.of(360));
            if (motorAngle.angle.lt(TurretConstants.MIN_ANGLE))
                motorAngle.angle = motorAngle.angle.plus(Degrees.of(360));
            Logger.recordOutput(getName() + "/angleAfter", motorAngle.angle);

            return motorAngle.angle.plus(initialDiff);

        }, () -> yawFeedForward);

    }

    public static Angle solveForTurret(Angle motorRotor, Angle cancoder)
    {

        Logger.recordOutput("Turret/motorPositionRaw", motorRotor);
        Logger.recordOutput("Turret/cancoderPositionRaw", cancoder);

        Angle motorRotorAdjusted = motorRotor.minus(TurretConstants.MOTOR_ZERO_OFFSET);
        Angle cancoderAdjusted = cancoder.minus(TurretConstants.CANCODER_ZERO_OFFSET);

        Logger.recordOutput("Turret/motorPosition", motorRotorAdjusted);
        Logger.recordOutput("Turret/cancoderPosition", cancoderAdjusted);

        double motorRotorRot = motorRotorAdjusted.in(Rotations);
        double cancoderRot = cancoderAdjusted.in(Rotations);

        // Step 1: Get the 'Magnet-Level' offset-corrected position
        // motorRotorRot should be the raw value from the Kraken
        double motorFrac = motorRotorRot % 1.0;
        if (motorFrac < 0)
            motorFrac += 1.0;

        // Step 2: Raw absolute reading from CANcoder
        double cancoderFrac = cancoderRot % 1.0;
        if (cancoderFrac < 0)
            cancoderFrac += 1.0;

        Logger.recordOutput("Turret/motorFrac", motorFrac);
        Logger.recordOutput("Turret/cancoderFrac", cancoderFrac);

        // Step 3: Calculate search bounds
        // Motor rotations at turret limits
        // We search through every possible "Turn" the motor could be on
        double motorAtMinTurret = TurretConstants.MIN_ANGLE.in(Rotations) * TurretConstants.GEAR_RATIO;
        double motorAtMaxTurret = TurretConstants.MAX_ANGLE.in(Rotations) * TurretConstants.GEAR_RATIO;

        int minWrap = (int) Math.floor(motorAtMinTurret - 1);
        int maxWrap = (int) Math.ceil(motorAtMaxTurret + 1);

        // Tolerance for bounds checking (accounts for sensor noise)
        // 0.01 rotations = ~3.6 degrees of turret movement
        double boundsTolerance = 0.01;

        // Step 4: Search for best match
        double bestTurretPos = 0;
        double bestError = Double.MAX_VALUE;

        for (int wrap = minWrap; wrap <= maxWrap; wrap++)
        {
            // Assume the motor is on turn 'wrap'
            double candidateMotor = motorFrac + wrap;
            double candidateTurret = candidateMotor / TurretConstants.GEAR_RATIO;

            // Skip if this hypothesis is physically impossible (with tolerance for noise)
            if (candidateTurret < TurretConstants.MIN_ANGLE.in(Rotations) - boundsTolerance ||
                    candidateTurret > TurretConstants.MAX_ANGLE.in(Rotations) + boundsTolerance)
            {
                continue;
            }

            // What would the CANcoder read if this turret position were true?
            double predictedCancoder = (candidateTurret * TurretConstants.CANCODER_GEAR_RATIO);
            double predictedFrac = predictedCancoder % 1.0;
            if (predictedFrac < 0)
                predictedFrac += 1.0;

            // Calculate circular error between the reality and the hypothesis
            double error = Math.abs(predictedFrac - cancoderFrac);
            if (error > 0.5)
                error = 1.0 - error;

            // Step 5: Update the best guess hypothesis
            if (error < bestError)
            {
                bestError = error;
                bestTurretPos = candidateTurret;
            }
        }

        // Clamp to actual bounds (in case tolerance allowed slightly out-of-bounds match)
        double minRot = TurretConstants.MIN_ANGLE.in(Rotations);
        double maxRot = TurretConstants.MAX_ANGLE.in(Rotations);
        if (bestTurretPos < minRot)
            bestTurretPos = minRot;
        if (bestTurretPos > maxRot)
            bestTurretPos = maxRot;

        Angle turretSeed = Rotations.of(bestTurretPos);

        Logger.recordOutput("Turret/solution", turretSeed);
        return turretSeed;
    }

}