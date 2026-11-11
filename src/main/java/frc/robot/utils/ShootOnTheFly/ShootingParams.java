// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.ShootOnTheFly;

import edu.wpi.first.math.interpolation.Interpolatable;

/** Add your docs here. */

public class ShootingParams implements Interpolatable<ShootingParams>
{
    public double RPM;
    public double angle;
    public double TOF;

    public ShootingParams(double RPM, double angle, double TOF)
    {
        this.RPM = RPM;
        this.angle = angle;
        this.TOF = TOF;
    }

    @Override
    public ShootingParams interpolate(ShootingParams other, double t)
    {

        t = Math.max(0.0, Math.min(1.0, t));
        return new ShootingParams(
                this.RPM + (other.RPM - this.RPM) * t,
                this.angle + (other.angle - this.angle) * t,
                this.TOF + (other.TOF - this.TOF) * t);
    }
}