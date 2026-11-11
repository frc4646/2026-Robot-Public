package frc.lib;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/*
 * Represents a specific RoboRIO, as a key for configurations.
 * 
 * The serial numbers here can be found on the label on the back: add a leading zero.
 * 
 * From Team 100
 */
public enum RobotIdentity
{
    BLACKFANG("023D2C46"),
    BLACKBEARD("032669F8"),
    BLANK(""), // e.g. test default or simulation
    UNKNOWN(null);

    private static final Map<String, RobotIdentity> identities = new HashMap<>();

    static
    {
        for (RobotIdentity i : RobotIdentity.values())
        {
            identities.put(i.m_serialNumber, i);
        }
    }

    public static final RobotIdentity instance = get();

    private final String m_serialNumber;

    private RobotIdentity(String serialNumber)
    {
        m_serialNumber = serialNumber;
    }

    private static RobotIdentity get()
    {
        String serialNumber = "";
        if (RobotBase.isReal())
        {
            // Calling getSerialNumber in a vscode unit test
            // SEGVs because it does the wrong
            // thing with JNIs, so don't do that.
            serialNumber = RobotController.getSerialNumber();
        }
        else
        {
            serialNumber = "";
        }
        if (identities.containsKey(serialNumber))
            return identities.get(serialNumber);
        return UNKNOWN;
    }
}