package frc.robot.subsystems.Flywheel;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.servomotor.ServoMotorSubsystem;
import frc.lib.servomotor.motors.ServoMotorTalonFX;
import frc.robot.RobotState;
import frc.robot.configuration.Constants;
import frc.robot.utils.FieldConstants_2026;
import frc.robot.utils.FieldUtil_2026;

/**
 * Subsystem for controlling the robot's arm using a servo motor.
 * Includes commands for moving the arm to specific positions and maintaining its current position.
 * Extends the ServoMotorSubsystem class with a ServoMotorTalonFX motor.
 */
public class FlywheelSubsystem extends ServoMotorSubsystem<ServoMotorTalonFX>
{
    public AngularVelocity startRPM;
    private RobotState robotState;
    private AngularVelocity shotVelo;

    public FlywheelSubsystem(RobotState robotState)
    {
        super(FlywheelConstants.getServoMotor());

        setCurrentPositionAsZero();
        this.robotState = robotState;
        shotVelo = RPM.of(0);
        // set default command to save current position, then hold it
        // setDefaultCommand(getServoCommands().setTargetDutyCycle(() -> 0));
        setDefaultCommand(setTargetRPM(RPM.of(1000)));
        startRPM = RPM.of(2000);

    }

    public AngularVelocity MetersToRPM(Supplier<Distance> distanceToHubMeters, boolean shooting)
    {
        // use FlywheelRPMMap to get a value from M/S
        double meters = distanceToHubMeters.get().in(Meters);
        double rpmDouble;
        if (shooting)
        {
            rpmDouble = Constants.SHOOTER_MAP.get(Meters.of(meters)).rpm().in(RPM);

        }
        else
        {
            rpmDouble = Constants.FEED_MAP.get(Meters.of(meters)).rpm().in(RPM);
        }
        AngularVelocity rpm = RPM.of(rpmDouble);
        return rpm;
    }

    public Command setShootingRPMFromDistance(Supplier<Distance> meters)
    {
        return setTargetRPM(() -> MetersToRPM(meters, true));
    }

    public Command setPassingRPMFromDistance(Supplier<Distance> meters)
    {
        return setTargetRPM(() -> MetersToRPM(meters, false));
    }

    public Command setRPMByActualDist()
    {
        return setTargetRPM(() -> shotVelo);
    }

    public Command setRPMByDistToAlliance()
    {
        return setTargetRPM(() -> shotVelo);
    }

    public void addRPM(int change)
    {
        AngularVelocity changeRPM = RPM.of(change);
        if (startRPM.plus(changeRPM).gte(FlywheelConstants.MAX_RPM))
        {
            startRPM = FlywheelConstants.MAX_RPM;
        }
        else if (startRPM.plus(changeRPM).lt(FlywheelConstants.MIN_SHOOTING_RPM))
        {
            startRPM = FlywheelConstants.MIN_SHOOTING_RPM;
        }
        else
        {
            startRPM = startRPM.plus(RPM.of(change));
        }
        Logger.recordOutput("Flywheel/startRPM", startRPM);
    }

    public Command testerCommand()
    {
        return setTargetRPM(() -> startRPM);
    }

    @Override
    public void periodic()
    {
        super.periodic();

        Logger.recordOutput(getName() + "/distance", FieldUtil_2026.distanceToTarget(robotState, FieldUtil_2026.chooseTarget(robotState)));
        Logger.recordOutput(getName() + "/rpm", MetersToRPM(() -> FieldUtil_2026.distanceToTarget(robotState, FieldUtil_2026.chooseTarget(robotState)), true));
        robotState.flywheel.setVelo(getActuals().getAngularVelocity());

        if (FieldUtil_2026.chooseTarget(robotState) == FieldConstants_2026.Hub.HubPose)
        {
            robotState.flywheel.setShootingVelo(MetersToRPM(() -> FieldUtil_2026.distanceToTarget(robotState, FieldUtil_2026.chooseTarget(robotState)), true));
        }
        else
        {
            robotState.flywheel.setShootingVelo(MetersToRPM(() -> FieldUtil_2026.distanceToTarget(robotState, FieldUtil_2026.chooseTarget(robotState)), false));
        }
        shotVelo = (robotState.calculateShotValues(FieldUtil_2026.chooseTarget(robotState), Constants.SHOOT_ON_THE_FLY.LATENCY_COMPENSATION).rpm());
        robotState.flywheel.setError(getErrorAngularVelocity());
    }

    public Command setTargetRPM(AngularVelocity targetRPM)
    {
        return setTargetRPM(() -> targetRPM);
    }

    public Command setTargetRPM(Supplier<AngularVelocity> targetRPM)
    {

        return getServoCommands().setTargetAngularVelocity(() ->
        {
            final var finalRPMObj = new Object()
            {
                AngularVelocity finalRPM = targetRPM.get();

            };
            if (targetRPM.get().lte(FlywheelConstants.MIN_RPM))
            {
                finalRPMObj.finalRPM = FlywheelConstants.MIN_RPM;

            }
            else if (targetRPM.get().gte(FlywheelConstants.MAX_RPM))
            {
                finalRPMObj.finalRPM = FlywheelConstants.MAX_RPM;

            }

            return finalRPMObj.finalRPM;
        });
    }
}
