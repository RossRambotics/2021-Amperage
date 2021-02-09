package frc.robot.helper;

public class ShooterValueSet // the values to be returned when a key is given to the lookup table
{
    public Double shooterRPM = null;
    public Double hoodAngle = null;

    ShooterValueSet(Double HoodAngle, Double ShooterRPM)
    {
        hoodAngle = HoodAngle;
        shooterRPM = ShooterRPM;
    }
}

