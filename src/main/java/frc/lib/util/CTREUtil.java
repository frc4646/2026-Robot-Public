package frc.lib.util;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.DriverStation;

/***
 * From Team 254
 */
public class CTREUtil
{
    public static final int MAX_RETRIES = 10;

    // thread to apply configs
    private static BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private static ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

    public static void tryUntilOK(Supplier<StatusCode> function, int deviceId)
    {

        threadPoolExecutor.submit(() ->
        {
            final int max_num_retries = 10;
            StatusCode statusCode = StatusCode.OK;
            for (int i = 0; i < max_num_retries; ++i)
            {
                statusCode = function.get();
                if (statusCode == StatusCode.OK)
                    break;
            }
            if (statusCode != StatusCode.OK)
            {
                DriverStation.reportError(
                        "Error calling " + function + " on ctre device id " + deviceId + ": " + statusCode, true);
            }
        });
    }

    public static void apply(TalonFX motor, TalonFXConfiguration config)
    {
        tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static void apply(TalonFX motor, SoftwareLimitSwitchConfigs config)
    {
        tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static void apply(TalonFX motor, CurrentLimitsConfigs config)
    {
        tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static void apply(TalonFXS motor, SoftwareLimitSwitchConfigs config)
    {
        tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static void apply(CANcoder cancoder, CANcoderConfiguration config)
    {
        tryUntilOK(() -> cancoder.getConfigurator().apply(config), cancoder.getDeviceID());
    }

    public static void apply(TalonFXS motor, TalonFXSConfiguration config)
    {
        tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static void apply(TalonFXS motor, CurrentLimitsConfigs config)
    {
        tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static void apply(TalonFX motor, SlotConfigs slotConfig)
    {
        tryUntilOK(() -> motor.getConfigurator().apply(slotConfig), motor.getDeviceID());
    }

    public static void refreshConfiguration(TalonFX motor, TalonFXConfiguration config)
    {
        tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
    }

    public static void refreshConfiguration(TalonFXS motor, TalonFXSConfiguration config)
    {
        tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
    }

    public static void refreshConfiguration(TalonFX motor, SoftwareLimitSwitchConfigs config)
    {
        tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
    }

    public static void refreshConfiguration(TalonFX motor, CurrentLimitsConfigs config)
    {
        tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
    }

    public static void refreshConfiguration(TalonFXS motor, SoftwareLimitSwitchConfigs config)
    {
        tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
    }

    public static void refreshConfiguration(TalonFXS motor, CurrentLimitsConfigs config)
    {
        tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
    }

}
