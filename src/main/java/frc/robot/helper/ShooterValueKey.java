package frc.robot.helper;

class ShooterValueKey // an look up value key with and its devation from the orignal distance
{
    public Double key = null;
    public Double devationFromDistance = null;

    ShooterValueKey(double Key, double DevationFromDistance) {
        key = Key;
        devationFromDistance = DevationFromDistance;
    }

}