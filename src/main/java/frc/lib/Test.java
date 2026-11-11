package frc.lib;

import java.util.List;

// import com.ctre.phoenix.CANifier;
// import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
// import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
// import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Test report system */
public class Test
{
    private static int numPass = 0, numFail = 0;

    /** Include in test report */
    public static void add(SubsystemBase subsystem, String testCriteria, boolean result)
    {
        countResult(result);
        System.out.println(String.format("%s %s: %s", (result) ? "Okay" : "ERROR", subsystem.getName(), testCriteria));
    }

    /** Include in test report */
    public static void add(String testCriteria, boolean result)
    {
        countResult(result);
        System.out.println(String.format("%s %s", (result) ? "Okay" : "ERROR", testCriteria));
    }

    /** Zero test report metrics */
    public static void reset()
    {
        banner();
        numPass = 0;
        numFail = 0;
    }

    /** Summarize test report */
    public static boolean results()
    {
        System.out.println("Pass: " + numPass + " Tests");
        System.out.println("Fail: " + numFail + " Tests");
        banner();
        return numFail == 0;
    }

    /**
     * Check if pneumantic control module detects wires are short. Power cycle robot
     * when shorted.
     */
    public static void checkSolenoid(SubsystemBase subsystem, DoubleSolenoid solenoid)
    {
        boolean result = !solenoid.isFwdSolenoidDisabled();
        add(subsystem, String.format("Solenoid %d/%d", solenoid.getFwdChannel(), solenoid.getRevChannel()), result);
    }

    // /** Check if motor controllers need a software update */
    // public static void checkFirmware(SubsystemBase subsystem, BaseMotorController device)
    // {
    //     int deviceID = device.getDeviceID(), actual = device.getFirmwareVersion(), expected = 0x1600; // 0x16 is 22
    //     add(subsystem, String.format("Device %d Firmware 0x%X, Expected 0x%X", deviceID, actual, expected),
    //             actual == expected);
    // }

    // /** Check if motor controllers need a software update */
    // public static void checkFirmware(SubsystemBase subsystem, CANifier device)
    // {
    //     int deviceID = device.getDeviceID(), actual = device.getFirmwareVersion(), expected = 0x1600; // 0x16 is 22
    //     add(subsystem, String.format("Device %d Firmware 0x%X, Expected 0x%X", deviceID, actual, expected),
    //             actual == expected);
    // }

    // /** Check if motor controllers need a software update */
    // public static void checkFirmware(SubsystemBase subsystem, CANSparkMax device)
    // {
    //     int deviceID = device.getDeviceId(), actual = device.getFirmwareVersion(), expected = 0x01050002;
    //     add(subsystem, String.format("Device %d Firmware 0x%X, Expected 0x%X", deviceID, actual, expected),
    //             actual == expected);
    // }

    // public static void checkStatusFrames(SubsystemBase subsystem, BaseMotorController device)
    // {
    //     List<Pair<String, StatusFrameEnhanced>> frames = List.of(
    //             Pair.of("General", StatusFrameEnhanced.Status_1_General),
    //             Pair.of("Feedback", StatusFrameEnhanced.Status_2_Feedback0),
    //             Pair.of("Quadrature", StatusFrameEnhanced.Status_3_Quadrature),
    //             Pair.of("Analog Temp Vbat", StatusFrameEnhanced.Status_4_AinTempVbat),
    //             Pair.of("Pulse Width", StatusFrameEnhanced.Status_8_PulseWidth),
    //             Pair.of("Motion Magic", StatusFrameEnhanced.Status_10_MotionMagic));
    //     for (Pair<String, StatusFrameEnhanced> frame : frames)
    //     {
    //         System.out.println(String.format("Device %d Status Frame %s: %d", device.getDeviceID(), frame.getFirst(),
    //                 device.getStatusFramePeriod(frame.getSecond())));
    //     }
    // }

    private static void countResult(boolean result)
    {
        if (result)
        {
            numPass++;
        }
        else
        {
            numFail++;
        }
    }

    private static void banner()
    {
        System.out.println("============================TESTING============================");
    }
}
