package frc.lib.util;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class UnitsMath
{

    /**
     * Calculate the Moment of Inertia from the (CAD) measured center of mass
     * 
     * @param mass
     *            total mass of the mechanism
     * @param centerOfMass
     *            distance from the pivot point to the center of mass
     * @return
     */
    public static MomentOfInertia CalcMomentOfInertia(Mass mass, Distance centerOfMass)
    {
        return KilogramSquareMeters.of(mass.in(Kilograms) * centerOfMass.in(Meters) * centerOfMass.in(Meters));
    }
}
