package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class DelayedBoolean
{
    private final double timeDelay;
    private double timeTransition;
    private boolean last;

    public DelayedBoolean(double delay)
    {
        timeDelay = delay;
        reset();
    }

    public boolean isSet(boolean current)
    {
        double time = Timer.getFPGATimestamp();
        boolean valueChanged = current && !last;
        boolean result = false;

        if (valueChanged)
        {
            timeTransition = time;
        }

        if (current && (time - timeTransition > timeDelay))
        {
            result = true;
        }

        last = current;
        return result;
    }

    public void reset()
    {
        timeTransition = Timer.getFPGATimestamp();
        last = false;
    }
}
