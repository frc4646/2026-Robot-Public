// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeArm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.servomotor.ServoMotorSubsystem;
import frc.lib.servomotor.ServoMotorViz;
import frc.lib.servomotor.ServoMotorViz.VizConfig;
import frc.lib.servomotor.motors.ServoMotorTalonFX;
import frc.robot.RobotState;

public class IntakeArmSubsystem extends ServoMotorSubsystem<ServoMotorTalonFX>
{
    private RobotState robotState;
    private Angle holdAngle;
    ServoMotorViz viz;

    /** Creates a new Intake. */
    public IntakeArmSubsystem(RobotState robotState)
    {

        super(IntakeArmConstants.getServoMotor());

        viz = ServoMotorViz.addPivot(getName() + "Intake Arm", IntakeArmConstants.ORIGIN,
                () -> getActuals().getAngle(), new VizConfig(Color.kHotPink, Degrees.of(90), IntakeArmConstants.ARM_LENGTH));

        this.robotState = robotState;

        setDefaultCommand(intakeDefaultCommand());
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        super.periodic();
        viz.update();
        robotState.intake.setArmAngle(this.getActuals().getAngle());

    }

    public Command deploy()
    {
        return getServoCommands().setTargetAngle(() -> IntakeArmConstants.FORWARD_MAX_POSITION, 0).withName("Deploy"); // must TUNE methinks
    }

    public Command undeploy()
    {
        return getServoCommands().setTargetAngle(() -> IntakeArmConstants.REVERSE_MIN_POSITION, 1).withName("Undeploy"); // also tune ROFL!
    }

    public Command intakeDefaultCommand()
    {
        return Commands.runOnce(() ->
        {
            holdAngle = getActuals().getAngle();
        }).andThen(getServoCommands().setTargetAngle(() -> holdAngle));
    }

    public Command jostleDeploy()
    {
        return getServoCommands().setTargetAngle(() -> (IntakeArmConstants.FORWARD_MAX_POSITION.minus(Degrees.of(20))), 0);
    }

    public Command jostleStow()
    {
        return getServoCommands().setTargetAngle(() -> (IntakeArmConstants.REVERSE_MIN_POSITION.plus(Degrees.of(50))), 1);
    }

    public Command TestDeploy()
    {
        return getServoCommands().setTargetDutyCycle(0.2);
    }

    public Command TestStow()
    {
        return getServoCommands().setTargetDutyCycle(-0.2);
    }
}
