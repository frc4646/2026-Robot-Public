package frc.lib.factories;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import frc.lib.util.CANDeviceId;
import frc.lib.util.CTREUtil;

@java.lang.SuppressWarnings(
{
        "java:S1104", "java:S116"
})

public class CANcoderFactory
{
    public static CANcoderConfiguration makeDefaultConfig(Angle zeroPositionOffsets)
    {
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

        cancoderConfig.MagnetSensor.MagnetOffset = zeroPositionOffsets.in(Rotations);
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        return cancoderConfig;
    }

    public static CANcoder makeDefault(CANDeviceId deviceId, Angle zeroPositionOffsets)
    {
        CANcoder cancoder = new CANcoder(deviceId.getId(), deviceId.getBus());
        CTREUtil.apply(cancoder, makeDefaultConfig(zeroPositionOffsets));
        return cancoder;
    }

    public static CANcoder make(CANDeviceId deviceId, CANcoderConfiguration config)
    {
        CANcoder cancoder = new CANcoder(deviceId.getId(), deviceId.getBus());
        CTREUtil.apply(cancoder, config);
        return cancoder;
    }
}
