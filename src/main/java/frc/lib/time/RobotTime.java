package frc.lib.time;

import org.littletonrobotics.junction.Logger;

/***
 * From Team 254
 */
public class RobotTime
{
    public static double getTimestampSeconds()
    {
        long micros = Logger.getTimestamp();
        return (double) micros * 1.0E-6;
    }
}
