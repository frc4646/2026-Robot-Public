package frc.lib.motor;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MotorActualsAutoLogged extends MotorActuals implements LoggableInputs, Cloneable
{
    @Override
    public void toLog(LogTable table)
    {
        table.put("VelocityRotationsPerSecond", velocityRotationsPerSecond);
        table.put("PositionRotations", positionRotations);
        table.put("VelocityInchesPerSecond", velocityInchesPerSecond);
        table.put("PositionInches", positionInches);
        table.put("AppliedVolts", appliedVolts);
        table.put("CurrentStatorAmps", currentStatorAmps);
        table.put("CurrentSupplyAmps", currentSupplyAmps);
        table.put("TorqueCurrentAmps", torqueCurrentAmps);
        table.put("SoftLimitForward", softLimitForward);
        table.put("SoftLimitReverse", softLimitReverse);
    }

    @Override
    public void fromLog(LogTable table)
    {
        velocityRotationsPerSecond = table.get("VelocityRotationsPerSecond", velocityRotationsPerSecond);
        positionRotations = table.get("PositionRotations", positionRotations);
        velocityInchesPerSecond = table.get("VelocityInchesPerSecond", velocityInchesPerSecond);
        positionInches = table.get("PositionInches", positionInches);
        appliedVolts = table.get("AppliedVolts", appliedVolts);
        currentStatorAmps = table.get("CurrentStatorAmps", currentStatorAmps);
        currentSupplyAmps = table.get("CurrentSupplyAmps", currentSupplyAmps);
        torqueCurrentAmps = table.get("TorqueCurrentAmps", torqueCurrentAmps);
        softLimitForward = table.get("SoftLimitForward", softLimitForward);
        softLimitReverse = table.get("SoftLimitReverse", softLimitReverse);
    }

    public MotorActualsAutoLogged clone()
    {
        MotorActualsAutoLogged copy = new MotorActualsAutoLogged();
        copy.velocityRotationsPerSecond = this.velocityRotationsPerSecond;
        copy.positionRotations = this.positionRotations;
        copy.velocityInchesPerSecond = this.velocityInchesPerSecond;
        copy.positionInches = this.positionInches;
        copy.appliedVolts = this.appliedVolts;
        copy.currentStatorAmps = this.currentStatorAmps;
        copy.currentSupplyAmps = this.currentSupplyAmps;
        copy.torqueCurrentAmps = this.torqueCurrentAmps;
        copy.softLimitForward = this.softLimitForward;
        copy.softLimitReverse = this.softLimitReverse;
        return copy;
    }
}
