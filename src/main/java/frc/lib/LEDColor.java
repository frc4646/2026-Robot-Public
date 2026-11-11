package frc.lib;

public class LEDColor
{
    public final int red, green, blue;

    public LEDColor()
    {
        this(0, 0, 0);
    }

    public LEDColor(int red, int green, int blue)
    {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    public boolean isEqual(LEDColor other)
    {
        return red == other.red && green == other.green && blue == other.blue;
    }

    public String getHex()
    {
        return String.format("#%02x%02x%02x", red, green, blue);
    }
}
