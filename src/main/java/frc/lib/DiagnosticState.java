package frc.lib;

public class DiagnosticState
{
    public final LEDColor color;
    public final boolean flash;

    public DiagnosticState(LEDColor diagnostic)
    {
        this(diagnostic, false);
    }

    public DiagnosticState(LEDColor diagnostic, boolean flash)
    {
        this.color = diagnostic;
        this.flash = flash;
    }
}
