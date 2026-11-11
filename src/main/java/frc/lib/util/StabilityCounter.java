package frc.lib.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A utility class to determine if a value has been stable for a certain number of iterations.
 * This is useful for filtering out noise in sensor readings or other fluctuating values.
 */
public class StabilityCounter
{
    private final int threshold;
    private int stableCounts = 0;
    private final BooleanSupplier isCurrentlyStable;

    /**
     * Constructs a StabilityCounter with a specified stable threshold.
     *
     * @param stableThreshold
     *            the number of consecutive stable counts required to consider the value stable
     */
    public StabilityCounter(int stableThreshold, BooleanSupplier isCurrentlyStable)
    {
        this.threshold = stableThreshold;
        this.isCurrentlyStable = isCurrentlyStable;
        reset();
    }

    /**
     * Increments the stable count if the current value is stable.
     * 
     * @return true if the value is stable, false otherwise.
     */
    public boolean calculate()
    {
        stableCounts++;

        if (!isCurrentlyStable.getAsBoolean())
        {
            reset();
        }
        return isStable();
    }

    /**
     * Resets the stable count to zero.
     */
    public void reset()
    {
        stableCounts = 0;
    }

    /**
     * Returns the number of times it's been stable
     * 
     * @return the number of times the value has been stable
     */
    public int counts()
    {
        return stableCounts;
    }

    /**
     * Checks if the value has been stable for the required number of counts.
     * 
     * @return true if the value is stable, false otherwise
     */
    public boolean isStable()
    {
        return stableCounts >= threshold;
    }

    /**
     * Creates a Trigger that will be active when the value is stable.
     * 
     * @return
     */
    public Trigger asTrigger()
    {
        return new Trigger(() -> isStable());
    }
}
