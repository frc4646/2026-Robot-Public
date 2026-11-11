package frc.robot.subsystems.TurretHood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.servomotor.ServoMotorSubsystem;
import frc.lib.servomotor.motors.ServoMotorTalonFX;
import frc.robot.RobotState;
import frc.robot.configuration.Constants;
import frc.robot.utils.FieldConstants_2026;
import frc.robot.utils.FieldUtil_2026;

public class TurretHoodSubsystem extends ServoMotorSubsystem<ServoMotorTalonFX>
{
    private Angle shotToTargetAngle;
    public Angle customAngle;
    private RobotState robotState;

    public TurretHoodSubsystem(RobotState robotState)
    {
        super(TurretHoodConstants.getServoMotor());
        setCurrentPosition(TurretHoodConstants.REVERSE_MIN_POSITION);

        shotToTargetAngle = TurretHoodConstants.MAX_DIS_SHOT_ANGLE;
        this.robotState = robotState;

        setDefaultCommand(getDefaultCommand());

        customAngle = Degrees.of(0);
    }

    @Override
    public void periodic()
    {
        super.periodic();

        Pose2d targetPose = FieldUtil_2026.chooseTarget(robotState);
        Distance distanceToTarget = FieldUtil_2026.distanceToTarget(robotState, targetPose);
        Logger.recordOutput(getName() + "/distanceToTarget", distanceToTarget);
        // Logger.recordOutput(getName() + "/targetPose", targetPose);

        if (FieldUtil_2026.chooseTarget(robotState) == FieldConstants_2026.Hub.HubPose)
        {
            shotToTargetAngle = distanceToShotAngle(FieldUtil_2026.distanceToTarget(robotState, targetPose));
        }
        else
        {
            shotToTargetAngle = distanceToFeedAngle(FieldUtil_2026.distanceToTarget(robotState, targetPose));
        }

        /* SOTF STEPS */

        shotToTargetAngle = robotState.calculateShotValues(targetPose, Constants.SHOOT_ON_THE_FLY.LATENCY_COMPENSATION).hoodAngle();

        if (!(robotState.safeToLiftHood()))
        {
            shotToTargetAngle = TurretHoodConstants.HoodAngleToShotAngle(TurretHoodConstants.UNDER_TRENCH_ANGLE);
        }
        robotState.hood.setHoodAngle(getActuals().getAngle());

        robotState.hood.setError(getErrorAngle().in(Degrees));
    }

    public Angle distanceToFeedAngle(Distance distanceToTarget)
    {

        if (distanceToTarget.lte(TurretHoodConstants.MIN_SHOOTING_DISTANCE))
        {
            return TurretHoodConstants.MIN_DIS_SHOT_ANGLE;
        }
        else if (distanceToTarget.gte(TurretHoodConstants.MAX_SHOOTING_DISTANCE))
        {
            return TurretHoodConstants.MAX_DIS_SHOT_ANGLE;
        }

        double result = Constants.FEED_MAP.get(distanceToTarget).angle().in(Degrees);
        Logger.recordOutput(getName() + "/targetDegrees", result);
        return (Degrees.of(result));
    }

    public Angle distanceToShotAngle(Distance distanceToTarget)
    {

        if (distanceToTarget.lte(TurretHoodConstants.MIN_SHOOTING_DISTANCE))
        {
            return TurretHoodConstants.MIN_DIS_SHOT_ANGLE;
        }
        else if (distanceToTarget.gte(TurretHoodConstants.MAX_SHOOTING_DISTANCE))
        {
            return TurretHoodConstants.MAX_DIS_SHOT_ANGLE;
        }

        double result = Constants.SHOOTER_MAP.get(distanceToTarget).angle().in(Degrees);
        // Logger.recordOutput(getName() + "/targetDegrees", result);
        return (Degrees.of(result));
    }

    public void offsetDeg(Angle change)
    {
        Angle changeDegree = change;
        if (customAngle.plus(changeDegree).gte(TurretHoodConstants.FORWARD_MAX_POSITION))
        {
            customAngle = TurretHoodConstants.FORWARD_MAX_POSITION;
        }
        else if (customAngle.plus(changeDegree).lt(TurretHoodConstants.REVERSE_MIN_POSITION))
        {
            customAngle = TurretHoodConstants.REVERSE_MIN_POSITION;
        }
        else
        {
            customAngle = customAngle.plus(changeDegree);
        }
    }

    public Angle getShotToTargetAngle()
    {
        return shotToTargetAngle;
    }

    public Command setToHubAngle()
    {
        return getServoCommands().setTargetAngle(() -> TurretHoodConstants.ShotAngleToHoodAngle(shotToTargetAngle)).alongWith(Commands.runOnce(() ->
        {
            Logger.recordOutput(getName() + "ShootingAtHub", true);
        }));
    }

    public Command getDefaultCommand()
    {
        return getServoCommands().setTargetAngle_UntilAtAngle(TurretHoodConstants.UNDER_TRENCH_ANGLE, Degrees.of(1))
                .withDeadline(new WaitCommand(0.5))
                .andThen(calibratePositionToMin().withDeadline(new WaitCommand(1.5)))
                .andThen(getServoCommands().setTargetAngle(TurretHoodConstants.UNDER_TRENCH_ANGLE));
    }

    private final DutyCycleOut calibrationRequest = new DutyCycleOut(-0.15)
            .withIgnoreHardwareLimits(true)
            .withIgnoreSoftwareLimits(true);

    /** Trigger to detect when the elevator drives into a hard stop. */
    public final Trigger isHardStop = new Trigger(() ->
    {
        return getActuals().getAngularVelocity().abs(RotationsPerSecond) < 1 &&
                getActuals().getTorqueCurrent().abs(Amps) > 5;
    }).debounce(0.04);

    /**
     * Recalibrates position to minimum angle.
     * This slowly drives down until we see a drop in velocity and a spike in torque current,
     * indicating that we've hit a hard stop, then sets the current position as the minimum angle.
     */
    public Command calibratePositionToMin()
    {
        return getServoCommands().setCustomRequest(calibrationRequest)
                .until(isHardStop).withTimeout(2)
                .andThen(
                        getServoCommands().setTargetDutyCycle(0.0).withTimeout(0.25)
                                .andThen(() ->
                                {
                                    setCurrentPosition(TurretHoodConstants.REVERSE_MIN_POSITION);
                                }));
    }

}
