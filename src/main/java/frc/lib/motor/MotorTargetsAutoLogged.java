package frc.lib.motor;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MotorTargetsAutoLogged extends MotorTargets implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Mode", mode);
    table.put("VelocityRotationsPerSecond", velocityRotationsPerSecond);
    table.put("VelocityInchesPerSecond", velocityInchesPerSecond);
    table.put("PositionRotations", positionRotations);
    table.put("PositionInches", positionInches);
    table.put("Voltage", voltage);
    table.put("DutyCycle", dutyCycle);
  }

  @Override
  public void fromLog(LogTable table) {
    mode = table.get("Mode", mode);
    velocityRotationsPerSecond = table.get("VelocityRotationsPerSecond", velocityRotationsPerSecond);
    velocityInchesPerSecond = table.get("VelocityInchesPerSecond", velocityInchesPerSecond);
    positionRotations = table.get("PositionRotations", positionRotations);
    positionInches = table.get("PositionInches", positionInches);
    voltage = table.get("Voltage", voltage);
    dutyCycle = table.get("DutyCycle", dutyCycle);
  }

  public MotorTargetsAutoLogged clone() {
    MotorTargetsAutoLogged copy = new MotorTargetsAutoLogged();
    copy.mode = this.mode;
    copy.velocityRotationsPerSecond = this.velocityRotationsPerSecond;
    copy.velocityInchesPerSecond = this.velocityInchesPerSecond;
    copy.positionRotations = this.positionRotations;
    copy.positionInches = this.positionInches;
    copy.voltage = this.voltage;
    copy.dutyCycle = this.dutyCycle;
    return copy;
  }
}
