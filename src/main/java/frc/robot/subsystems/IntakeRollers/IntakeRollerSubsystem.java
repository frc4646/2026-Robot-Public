// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeRollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.lib.servomotor.ServoMotorSubsystem;
import frc.lib.servomotor.motors.ServoMotorTalonFX;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

public class IntakeRollerSubsystem extends ServoMotorSubsystem<ServoMotorTalonFX>
{
    /** Creates a new IntakeRollerSubsystem. */
    private RobotState robotState;

    public IntakeRollerSubsystem(RobotState robotState)
    {

        super(IntakeRollerConstants.getServoMotor());

        this.setDefaultCommand(getServoCommands().setTargetDutyCycle(0));
        this.robotState = robotState;
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        super.periodic();
    }

    public Command spinCommand()
    {
        return new ConditionalCommand(
                getServoCommands().setTargetAngularVelocity(() -> IntakeRollerConstants.INTAKE_SPIN_SPEED_AUTO),
                getServoCommands().setTargetAngularVelocity(() -> IntakeRollerConstants.INTAKE_SPIN_SPEED_AUTO),
                RobotContainer.isAutonomous());
    }

    public Command spinIfCanIntakeCommand()
    {
        return new ConditionalCommand(
                getServoCommands()
                        .setTargetAngularVelocity(() -> IntakeRollerConstants.INTAKE_SPIN_SPEED_AUTO.times(robotState.safeToLiftHopperHood() ? 1.0 : 0)),
                getServoCommands()
                        .setTargetAngularVelocity(() -> IntakeRollerConstants.INTAKE_SPIN_SPEED_AUTO.times(robotState.safeToLiftHopperHood() ? 1.0 : 0)),
                RobotContainer.isAutonomous());
    }

    public Command stopCommand()
    {
        return new ConditionalCommand(getServoCommands().setTargetDutyCycle(() -> IntakeRollerConstants.INTAKE_STOP_SPEED_AUTO),
                getServoCommands().setTargetVoltage(() -> IntakeRollerConstants.INTAKE_STOP_SPEED_TELE),
                RobotContainer.isAutonomous());
    }

    public Command ejectCommand()
    {
        return new ConditionalCommand(getServoCommands().setTargetDutyCycle(() -> IntakeRollerConstants.INTAKE_EJECT_SPEED_AUTO),
                getServoCommands().setTargetVoltage(() -> IntakeRollerConstants.INTAKE_EJECT_SPEED_TELE),
                RobotContainer.isAutonomous());
    }

}
