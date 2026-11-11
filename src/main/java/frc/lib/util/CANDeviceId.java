package frc.lib.util;

import com.ctre.phoenix6.CANBus;

/***
 * From Team 254
 */
public class CANDeviceId
{
    private final int id;
    private final CANBus bus;

    public CANDeviceId(int id, CANBus bus)
    {
        this.id = id;
        this.bus = bus;
    }

    /**
     * Constructs a CANDeviceId with the given ID and bus name.
     * 
     * @param id
     * @param bus
     * 
     * @deprecated Construct devices using a {@link CANBus} instance instead.
     */
    @Deprecated
    public CANDeviceId(int id, String bus)
    {
        this.id = id;
        this.bus = new CANBus(bus);
    }

    /**
     * Returns the Address/Id of the CAN device.
     * 
     * @return
     */
    public int getId()
    {
        return id;
    }

    /**
     * Returns the bus of the CAN device.
     * 
     * @return
     */
    public CANBus getBus()
    {
        return bus;
    }

    public boolean equals(CANDeviceId other)
    {
        return other.id == id && other.bus == bus;
    }
}