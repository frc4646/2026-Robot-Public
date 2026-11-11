package frc.lib.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class SwitchingCommand extends Command
{
    private final Command commandWhileTrue;
    private final Command commandWhileFalse;
    private final BooleanSupplier condition;
    private boolean wasTrue;

    public SwitchingCommand(BooleanSupplier condition, Command commandWhileTrue, Command commandWhileFalse)
    {
        this.condition = condition;
        this.commandWhileTrue = commandWhileTrue;
        this.commandWhileFalse = commandWhileFalse;
        addRequirements(commandWhileTrue.getRequirements());
        addRequirements(commandWhileFalse.getRequirements());
    }

    @Override
    public void initialize()
    {
        wasTrue = condition.getAsBoolean();
        if (wasTrue)
        {
            commandWhileTrue.initialize();
        }
        else
        {
            commandWhileFalse.initialize();
        }
    }

    @Override
    public void execute()
    {
        boolean isTrue = condition.getAsBoolean();

        if (isTrue != wasTrue)
        {
            if (wasTrue)
            {
                commandWhileTrue.end(true);
                commandWhileFalse.initialize();
            }
            else
            {
                commandWhileFalse.end(true);
                commandWhileTrue.initialize();
            }
            wasTrue = isTrue;
        }

        if (isTrue)
        {
            commandWhileTrue.execute();
        }
        else
        {
            commandWhileFalse.execute();
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        if (wasTrue)
        {
            commandWhileTrue.end(interrupted);
        }
        else
        {
            commandWhileFalse.end(interrupted);
        }
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}