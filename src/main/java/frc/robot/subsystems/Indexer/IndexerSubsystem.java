package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.RobotIdentity;
import frc.lib.servomotor.ServoMotorSubsystem;
import frc.lib.servomotor.ServoMotorViz;
import frc.lib.servomotor.motors.ServoMotorTalonFX;
import frc.robot.RobotState;
import frc.robot.configuration.Constants;
import frc.robot.utils.FieldUtil_2026;

public class IndexerSubsystem extends SubsystemBase
{
    ServoMotorSubsystem<ServoMotorTalonFX> feeder; // feeds to shooter
    ServoMotorSubsystem<ServoMotorTalonFX> spindexer; // part of indexer that separates fuel
    ServoMotorSubsystem<ServoMotorTalonFX> belt_wall;

    private RobotState robotState;

    public final CANdi candi;

    private static final String LIFETIME_BALLS_SHOT_KEY = "lifetimeBallsShot";
    private static final String LIFETIME_BALLS_SHOT_AT_HUB_KEY = "lifetimeBallsShotAtHub";

    protected ServoMotorViz vizFeeder;
    protected ServoMotorViz vizSPINDEXER;
    protected boolean hasFuelNear;
    protected Distance fuelDistance;
    protected CANrange canRange;

    private double valueSpindexer;
    private double valueBeltWall;
    private double valueFeeder;
    int ballsShot = 0;
    int lifetimeBallsShot = 0;
    int lifetimeBallsShotAtHub = 0;
    double ballsPerSocond = 0.0;
    boolean wasDetected = false;
    boolean wasShooting = false;
    double old_time;
    double time_passed = 0;
    double shootingStartTime = 0;
    // Lid timers v
    double lidOpenTime = 0.0;
    double lidClosedTime = 0.0;
    double lidMoveTime = 0.0;
    boolean wasLidClosed = false;
    double lidDebounceTime = 0.25;

    double timeInState = 0.0;

    double shootingAttemptStartTime = 0;
    boolean wasShootingForJam = false;

    boolean beltWallJam = false;
    boolean spindexerJam = false;
    boolean feederJam = false;

    public IndexerSubsystem(RobotState robotState)
    {
        candi = new CANdi(Constants.CAN.CANDI.getId(), Constants.CAN.CANDI.getBus());

        canRange = new CANrange(Constants.CAN.FEEDER_CAN_RANGE.getId(), Constants.CAN.FEEDER_CAN_RANGE.getBus());
        var proxConfig = new ProximityParamsConfigs();
        proxConfig.ProximityThreshold = IndexerConstants.NO_FUEL_DISTANCE.in(Meter);
        proxConfig.ProximityHysteresis = Inch.of(.5).in(Meter);
        canRange.getConfigurator().apply(proxConfig);

        lifetimeBallsShot = Preferences.getInt(LIFETIME_BALLS_SHOT_KEY, 0);
        lifetimeBallsShotAtHub = Preferences.getInt(LIFETIME_BALLS_SHOT_AT_HUB_KEY, 0);

        feeder = new ServoMotorSubsystem<ServoMotorTalonFX>(IndexerConstants.getServoMotorFeeder());
        spindexer = new ServoMotorSubsystem<ServoMotorTalonFX>(IndexerConstants.getServoMotorSPINDEXER());
        belt_wall = new ServoMotorSubsystem<ServoMotorTalonFX>(IndexerConstants.getServoMotorBELTWALL());

        this.robotState = robotState;

        setDefaultCommand(indexerDefaultCommand());
        SmartDashboard.putNumber("SpindexerSpeed", 0.75);
        SmartDashboard.putNumber("BeltWallSpeed", 0.75);
        SmartDashboard.putNumber("FeederSpeed", 0.75);
    }

    public void periodic()
    {
        feeder.periodic();
        spindexer.periodic();
        // hasFuelNear = canRange.getDistance().getValue().lt(IndexerConstants.NO_FUEL_DISTANCE);
        // feederHasFuelSpinCommand(); // add in fancy math idk
        // hello = canRange.getIsDetected()
        // fuelDistance = canRange.getDistance().getValue();
        valueSpindexer = SmartDashboard.getNumber("SpindexerSpeed", 0);
        valueFeeder = SmartDashboard.getNumber("FeederSpeed", 0);
        valueBeltWall = SmartDashboard.getNumber("BeltWallSpeed", 0);
        Logger.recordOutput(getName() + "/canRageValue", canRange.getDistance().getValueAsDouble());
        Logger.recordOutput(getName() + "/LifetimeBallsShot", lifetimeBallsShot);
        Logger.recordOutput(getName() + "/LifetimeBallsShotAtHub", lifetimeBallsShotAtHub);
        // For Jam detection v
        // Logger.recordOutput(getName() + "/IsIndexingJammed", isSpindexerJammed);
        Logger.recordOutput(getName() + "/JamElapsed", Timer.getFPGATimestamp() - shootingAttemptStartTime);

        Logger.recordOutput(getName() + "/Indexing/Spindexer/Velocity", spindexer.getActuals().getAngularVelocity().abs(RotationsPerSecond));
        Logger.recordOutput(getName() + "/Indexing/Spindexer/Current", spindexer.getActuals().getTorqueCurrent().abs(Amps));
        Logger.recordOutput(getName() + "/Indexing/SpindexerJammed", spindexerJam);

        Logger.recordOutput(getName() + "/Indexing/BeltWall/Velocity", belt_wall.getActuals().getAngularVelocity().abs(RotationsPerSecond));
        Logger.recordOutput(getName() + "/Indexing/BeltWall/Current", belt_wall.getActuals().getTorqueCurrent().abs(Amps));
        Logger.recordOutput(getName() + "/Indexing/BeltWallJammed", beltWallJam);

        Logger.recordOutput(getName() + "/Indexing/Feeder/Velocity", feeder.getActuals().getAngularVelocity().abs(RotationsPerSecond));
        Logger.recordOutput(getName() + "/Indexing/Feeder/Current", feeder.getActuals().getTorqueCurrent().abs(Amps));
        Logger.recordOutput(getName() + "/Indexing/FeederJammed", feederJam);
        // For Jam detection ^

        Logger.recordOutput("/Indexing/WasLidOpen", wasLidClosed);
        Logger.recordOutput("/Indexing/getLidClosedTime", getLidClosedTime());
        Logger.recordOutput("/Indexing/getLidOpenTime", getLidOpenTime());

        Logger.recordOutput(getName() + "/isLidClosed", isLidClosed());
        Logger.recordOutput(getName() + "/candiGetS1StateValue", candi.getS1State().getValue().value);

        boolean currentDetected = canRange.getIsDetected().getValue();
        if (currentDetected)
        {
            old_time = Timer.getFPGATimestamp();
        }
        time_passed = Timer.getFPGATimestamp() - old_time;
        robotState.timeSinceLastShot = time_passed;

        // For Anti-Jam logic v
        if (robotState.swerve.getShooting() && !wasShootingForJam)
        {
            shootingAttemptStartTime = Timer.getFPGATimestamp();
        }
        wasShootingForJam = robotState.swerve.getShooting();

        // Counting BPS v
        if (robotState.swerve.getShooting())
        {

            if (!wasShooting)
            { // reset on new shooting
                ballsShot = 0;
            }

            if (currentDetected && !wasDetected)
            { // Counts balls while shooting
                if (ballsShot == 0)
                {
                    shootingStartTime = Timer.getFPGATimestamp();
                }
                ballsShot++;
                lifetimeBallsShot++;
                if (FieldUtil_2026.isInAllianceZone(robotState))
                {
                    lifetimeBallsShotAtHub++;
                }
                Preferences.setInt(LIFETIME_BALLS_SHOT_KEY, lifetimeBallsShot); // Adding +1 to how many balls shot
                Preferences.setInt(LIFETIME_BALLS_SHOT_AT_HUB_KEY, lifetimeBallsShotAtHub);
            }

            double elapsed = Timer.getFPGATimestamp() - shootingStartTime;
            if (time_passed > 0)
            { // Counts the BPS if the time passed is more then 0
                ballsPerSocond = ballsShot / elapsed;
                Logger.recordOutput(getName() + "/BallsPerSecond", ballsPerSocond);
                Logger.recordOutput(getName() + "/ElapsedTime", elapsed);
            }
            wasDetected = currentDetected;
            wasShooting = true;
        }
        else
        {
            wasShooting = false;
        }

        robotState.indexer.setIsLidClosed(isLidClosed());

        SmartDashboard.putNumber("Balls Shot", ballsShot);

        boolean lidClosed = isLidClosed();

        // Lid just changed state — reset timer
        if (lidClosed != wasLidClosed)
        {
            lidMoveTime = Timer.getFPGATimestamp();
        }

        timeInState = Timer.getFPGATimestamp() - lidMoveTime;
        Logger.recordOutput("timeInState", timeInState);

        wasLidClosed = lidClosed;
    }

    public Command indexerDefaultCommand()
    {
        return run(() ->
        {
            feeder.getServoCommands().setTargetDutyCycle(0);
            spindexer.getServoCommands().setTargetDutyCycle(0);
            belt_wall.getServoCommands().setTargetDutyCycle(0);
        });
    }

    public Command dashboardSetSpeedSpindexer()
    {
        return spindexer.getServoCommands().setTargetDutyCycle(() -> valueSpindexer);
    }

    public Command dashboardSetSpeedFeeder()
    {
        return feeder.getServoCommands().setTargetDutyCycle(() -> valueFeeder);
    }

    public Command dashboardSetSpeedBeltWall()
    {
        return belt_wall.getServoCommands().setTargetDutyCycle(() -> valueBeltWall);
    }

    public Command dashboardSetSpeedIndexer()
    {
        return Commands.parallel(dashboardSetSpeedFeeder(), dashboardSetSpeedSpindexer(), dashboardSetSpeedBeltWall())
                .withName(getName() + "Index Dashboard Speed Spin Command");
    }

    public Command feederHasFuelSpinCommand()
    {
        return Commands.parallel(dashboardSetSpeedFeeder(), dashboardSetSpeedSpindexer().withName(getName() + "Index Spin Command Fuel Near")
                .onlyWhile(() -> hasFuelNear));
        // if fuel near make the feeder spin more with fancy math sick

    }

    public Command feederDoesntHaveFuelSpinCommand()
    {
        return feeder.getServoCommands().setTargetDutyCycle(valueFeeder).unless(() -> hasFuelNear); // tune speed
        // if fuel NOT near make the feeder spin same speed spindexer
    }

    public Command spindexerSpeedWhenNoFuelNearFeeder()
    {
        return feeder.getServoCommands().setTargetDutyCycle(valueSpindexer).unless(() -> hasFuelNear); // tune speed
        // if fuel NOT near make the spindexer spin same speed as feeder
    }

    public Command fuelNotNearIndexerCommand()
    {
        return Commands.parallel(feederDoesntHaveFuelSpinCommand(), spindexerSpeedWhenNoFuelNearFeeder().withName(getName() + "Index Spin No Fuel Near"));
        // indexer spins at same speed when no fuel is near enough
    }

    // public Command setIndexerSpeed() {
    // return Commands.parallel();
    // }

    // Change so you Tell Feeder to move x and Spindexer moves as a result of that
    public Command spinSpindexerWITHfeeder(double Speed)
    {
        double fSpeed = (Speed / IndexerConstants.FeederToSpindexer);
        return Commands.parallel(spindexer.getServoCommands().setTargetDutyCycle(Speed), feeder.getServoCommands().setTargetDutyCycle(fSpeed));
    }

    public Command spindexerSpinCommand()
    {
        return spindexer.getServoCommands().setTargetDutyCycle(() -> IndexerConstants.SPINDEXER_SPEED_BLACKBEARD); // TODO tune heh
    }

    public Command feederSpinCommand()
    {
        return feeder.getServoCommands().setTargetDutyCycle(() -> IndexerConstants.FEEDER_SPEED_BLACKBEARD); // TODO tune
    }

    public Command indexerSpinSpeed()
    {
        return Commands.parallel(spindexerSpinCommand(), feederSpinCommand().withName(getName() + "Index Spin Command"));
    }

    public Command indexAndFeedCommand(double spindexerSpeed, double feederSpeed)
    {
        return Commands.parallel(spindexer.getServoCommands().setTargetDutyCycle(spindexerSpeed),
                feeder.getServoCommands().setTargetDutyCycle(feederSpeed));
    }

    public Command indexAndFeedCommand(double spindexerSpeed, double feederSpeed, double beltWallSpeed)
    {
        return Commands.parallel(spindexer.getServoCommands().setTargetDutyCycle(spindexerSpeed),
                feeder.getServoCommands().setTargetDutyCycle(feederSpeed),
                belt_wall.getServoCommands().setTargetDutyCycle(beltWallSpeed));
    }

    public Command indexAndFeedCommand(DoubleSupplier spindexerSpeed, DoubleSupplier feederSpeed, DoubleSupplier beltWallSpeed)
    {
        return Commands.parallel(spindexer.getServoCommands().setTargetDutyCycle(spindexerSpeed),
                feeder.getServoCommands().setTargetDutyCycle(feederSpeed),
                belt_wall.getServoCommands().setTargetDutyCycle(beltWallSpeed));
    }

    public Command indexAndFeedCommand(Supplier<AngularVelocity> spindexerSpeed, Supplier<AngularVelocity> feederSpeed, Supplier<AngularVelocity> beltWallSpeed)
    {
        return Commands.parallel(spindexer.getServoCommands().setTargetAngularVelocity(spindexerSpeed),
                feeder.getServoCommands().setTargetAngularVelocity(feederSpeed),
                belt_wall.getServoCommands().setTargetAngularVelocity(beltWallSpeed));
    }

    public Command indexerSpecificRobotCommand()
    {
        return new ConditionalCommand(
                indexAndFeedCommand(IndexerConstants.SPINDEXER_SPEED_REDBEAN, IndexerConstants.FEEDER_SPEED_REDBEAN, IndexerConstants.BELT_WALL_SPEED_REDBEAN),
                indexAndFeedCommand(IndexerConstants.SPINDEXER_SPEED_BLACKBEARD, IndexerConstants.FEEDER_SPEED_BLACKBEARD),
                () -> RobotIdentity.instance == RobotIdentity.BLACKFANG); // default to blackbeard constants
    }

    public Command indexIfCanShoot()
    {
        return indexAndFeedCommand(() -> (IndexerConstants.SPINDEXER_SPEED_REDBEAN * (this.robotState.canShoot() ? 1.0 : 0)),
                (() -> IndexerConstants.FEEDER_SPEED_REDBEAN * (this.robotState.canShoot() ? 1.0 : 0)),
                (() -> IndexerConstants.BELT_WALL_SPEED_REDBEAN * (this.robotState.canShoot() ? 1.0 : 0)));
    }

    public Command indexIfCanShootAngularVelocity()
    {
        return indexAndFeedCommand(() -> (IndexerConstants.SPINDEXER_FEED_VELOCITY.times((this.robotState.canShoot() ? 1.0 : 0))),
                (() -> IndexerConstants.FEEDER_FEED_VELOCITY.times((this.robotState.canShoot() ? 1.0 : 0))),
                (() -> IndexerConstants.BELT_WALL_FEED_VELOCITY.times((this.robotState.canShoot() ? 1.0 : 0))));
    }

    public Command unIndex()
    {
        return indexAndFeedCommand((() -> (IndexerConstants.SPINDEXER_FEED_VELOCITY.times(-0.75))),
                (() -> IndexerConstants.FEEDER_FEED_VELOCITY.times(-0.75)),
                (() -> IndexerConstants.BELT_WALL_FEED_VELOCITY.times(-0.75))).alongWith(Commands.run(() -> robotState.timeSinceLastShot = 0));
    }

    public int getBallsShot()
    {
        return ballsShot;
    }

    // Jam detection v

    public final Trigger isSpindexerJammed = new Trigger(() ->
    {
        if (!robotState.swerve.getShooting())
        {
            beltWallJam = false;
            spindexerJam = false;
            feederJam = false;
            return false; // Return false if not even trying to shoot
        }

        double elapsed = Timer.getFPGATimestamp() - shootingAttemptStartTime;

        // All for advantageScope checks v

        spindexerJam = ((spindexer.getActuals().getAngularVelocity().abs(RPM) < 1.0)
                && (spindexer.getActuals().getTorqueCurrent().abs(Amps) > (((IndexerConstants.SpindexerStatorCurrentLimit) * .80))));
        beltWallJam = ((belt_wall.getActuals().getAngularVelocity().abs(RPM) < 1.0
                && belt_wall.getActuals().getTorqueCurrent().abs(Amps) > (((IndexerConstants.BeltWallStatorCurrentLimit) * .80))));
        feederJam = ((feeder.getActuals().getAngularVelocity().abs(RPM) < 1.0
                && feeder.getActuals().getTorqueCurrent().abs(Amps) > (((IndexerConstants.FeederStatorCurrentLimit) * .80))));

        return (elapsed > 1.0)
                &&
                ((spindexer.getActuals().getAngularVelocity().abs(RPM) < ((IndexerConstants.SPINDEXER_FEED_VELOCITY.in(RPM) * .10))
                        && spindexer.getActuals().getTorqueCurrent().abs(Amps) > ((IndexerConstants.SpindexerStatorCurrentLimit) * .5))
                        ||
                        (belt_wall.getActuals().getAngularVelocity().abs(RPM) < ((IndexerConstants.BELT_WALL_FEED_VELOCITY.in(RPM) * .10))
                                && belt_wall.getActuals().getTorqueCurrent().abs(Amps) > ((IndexerConstants.BeltWallStatorCurrentLimit) * .5))
                        ||
                        (feeder.getActuals().getAngularVelocity().abs(RPM) < ((IndexerConstants.FEEDER_FEED_VELOCITY.in(RPM) * .10))
                                && feeder.getActuals().getTorqueCurrent().abs(Amps) > ((IndexerConstants.FeederStatorCurrentLimit) * 0.5)))

                && (spindexer.getActuals().getAngularVelocity().gte(RPM.of(0)))
                && robotState.canShoot(); // only care about jams when we're trying to feed
    }).debounce(0.1);

    public Command antiJamCommand()
    { // Has timeouts just incase it trys to unjam forever or ball wont unjam
        return Commands.sequence(
                unIndex().withTimeout(.25),
                indexerSpecificRobotCommand().withTimeout(0.3)).repeatedly()
                .withTimeout(5.0)
                .withName(getName() + " Anti-Jam");
    }

    public boolean isLidClosed()
    {
        if (candi.getS1State().getValue().value == 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public double getLidOpenTime()
    {
        if (!wasLidClosed)
        {
            return timeInState;
        }
        else
        {
            return -1;
        }
    }

    public double getLidClosedTime()
    {
        if (wasLidClosed)
        {
            return timeInState;
        }
        else
        {
            return -1;
        }
    }

    public final Trigger caving = new Trigger(() ->
    {
        return (!isLidClosed()
                && robotState.timeSinceLastShot > IndexerConstants.SHOOTING_TIMEOUT
                && !isSpindexerJammed.getAsBoolean());
    }).debounce(0.1);
}
