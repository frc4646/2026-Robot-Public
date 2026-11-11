// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

public class LEDsSubsystem extends SubsystemBase
{

    public static final Color DISABLED_COLOR = Color.kWhite;
    public static final Color ENABLED_COLOR = Color.kRed;

    /** Creates a new LEDs. */
    private AddressableLED LEDs;
    private AddressableLEDBuffer LEDBuffer;
    private LEDPattern pattern;
    private Color color;
    private AddressableLEDSim LEDsim;
    private RobotState robotState;

    public LEDsSubsystem(RobotState robotState)
    {
        super("LEDs");
        this.robotState = robotState;
        LEDs = new AddressableLED(9);
        LEDBuffer = new AddressableLEDBuffer(52);
        LEDs.setLength(LEDBuffer.getLength());
        LEDs.setData(LEDBuffer);
        LEDs.start();

        pattern = LEDPattern.solid(CorrectColor(DISABLED_COLOR));

        if (RobotBase.isSimulation())
        {
            LEDsim = new AddressableLEDSim(LEDs);
            LEDsim.setRunning(true);
        }
    }

    private void applyPattern(LEDPattern pattern)
    {
        pattern.applyTo(LEDBuffer);
        LEDs.setData(LEDBuffer);
    }

    private Color CorrectColor(Color original)
    {
        if (Robot.isSimulation())
            return original;

        return new Color(original.blue, original.green, original.red);
    }

    public void solidColor(Color color)
    {
        pattern = LEDPattern.solid(CorrectColor(color));
        this.color = color;
        applyPattern(pattern);
    }

    public void solidCurrentColor()
    {
        pattern = LEDPattern.solid(CorrectColor(color));
        applyPattern(pattern);
    }

    public void gradientColor(Color colorOne, Color colorTwo)
    {
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, CorrectColor(colorOne), CorrectColor(colorTwo));
        applyPattern(pattern);
    }

    public void blinkingCurrentColor()
    {
        LEDPattern base = LEDPattern.solid(CorrectColor(color));
        pattern = base.blink(Seconds.of(.25));
        applyPattern(pattern);
    }

    public void blinkingCustomColor(Color color)
    {
        LEDPattern base = LEDPattern.solid(CorrectColor(color));
        pattern = base.blink(Seconds.of(.25));
        applyPattern(pattern);
    }

    public void blinkingCustomColor(Color color, double time)
    {
        LEDPattern base = LEDPattern.solid(CorrectColor(color));
        pattern = base.blink(Seconds.of(time));
        applyPattern(pattern);
    }

    public void progressBar(Color color)
    {
        // Color might work?? Double check progressBar code
        LEDPattern base = LEDPattern.solid(CorrectColor(color));
        LEDPattern mask = LEDPattern.progressMaskLayer(() -> 20);
        pattern = base.mask(mask);
        applyPattern(pattern);
    }

    public void progressBarNoColor()
    {
        pattern = LEDPattern.progressMaskLayer(() -> 20);
        applyPattern(pattern);
    }

    public void breathing(Color color)
    {
        LEDPattern base = LEDPattern.solid(CorrectColor(color));
        pattern = base.breathe(Seconds.of(1.5));
        applyPattern(pattern);
    }

    public void scrolling(Color colorOne, Color colorTwo)
    {
        LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, CorrectColor(colorOne), CorrectColor(colorTwo));
        pattern = base.scrollAtRelativeSpeed(Percent.per(Seconds).of(175));
        applyPattern(pattern);
    }

    @Override
    public void periodic()
    {
        // pattern.applyTo(LEDBuffer);
        // This method will be called once per scheduler run
        double timeUntilActive = robotState.getTimeUntilActiveShift();
        double timeUntilInactive = robotState.getTimeUntilInactiveShift();
        boolean isActiveShift = robotState.isActiveShift();
        boolean wonAuto = robotState.wonAutonomous();
        boolean isLidClosed = robotState.indexer.getIsLidClosed();

        Logger.recordOutput(getName() + "/Time until active", timeUntilActive);
        Logger.recordOutput(getName() + "/Time until inactive", timeUntilInactive);
        Logger.recordOutput(getName() + "/Is active shift", isActiveShift);
        Logger.recordOutput(getName() + "/Won auto", wonAuto);
        Logger.recordOutput(getName() + "/isLidCLosed", isLidClosed);

        if (DriverStation.isDisabled())
        {
            if (wonAuto && DriverStation.isFMSAttached() && DriverStation.isAutonomous())
            {
                Logger.recordOutput(getName() + "/Mode", "WonAuto");
                // TODO
                // solidColor(DISABLED_COLOR);
                blinkingCustomColor(DISABLED_COLOR, 0.2);
            }
            else
            {
                Logger.recordOutput(getName() + "/Mode", "Disabled");
                solidColor(DISABLED_COLOR);
            }
        }
        else
        {
            if (isActiveShift)
            {
                if (timeUntilInactive > 5)
                {
                    Logger.recordOutput(getName() + "/Mode", "Active");
                    if (!isLidClosed)
                    {
                        scrolling(RobotContainer.isOnRedAlliance() ? Color.kRed : Color.kBlue, Color.kMediumPurple);
                    }
                    else
                    {
                        solidColor(RobotContainer.isOnRedAlliance() ? Color.kRed : Color.kBlue);
                    }
                }
                else
                {
                    Logger.recordOutput(getName() + "/Mode", "ActiveEnding");
                    matchCountLEDs(timeUntilInactive);
                }
            }
            else
            {
                if (timeUntilActive > 5)
                {
                    Logger.recordOutput(getName() + "/Mode", "Inactve");
                    if (!isLidClosed)
                    {
                        scrolling(Color.kYellow, Color.kBlack);
                    }
                    else
                    {
                        solidColor(Color.kYellow);
                    }
                }
                else
                {
                    Logger.recordOutput(getName() + "/Mode", "InactiveEnding");
                    matchCountLEDs(timeUntilActive);
                }
            }
        }
        // // finally set the LEDs
        // LEDs.setData(LEDBuffer);
    }

    public Command runPattern(LEDPattern pattern)
    {
        return run(() ->
        {
            applyPattern(pattern);
        });
    }

    public void matchCountLEDs(Double time)
    {
        LEDPattern pattern = LEDPattern.solid(Color.kWhite);
        LEDPattern progressPattern = pattern.mask(LEDPattern.progressMaskLayer(() -> time / 5.0));
        applyPattern(progressPattern);
    }

}
