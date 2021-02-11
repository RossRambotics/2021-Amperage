package frc.robot.helper.DriveHandlingSetup;

public class DefaultHardSurfaceHandling extends HandlingBase {
    @Override
    public double getMaxDriveOutputInitialValue() {
        return 0.5;
    }

    @Override
    public String getDefaultDriveCommandNameInitialValue() {
        return "TankDriveHandBrake";
    }

    @Override
    public String getTabName() {
        return "HardSurfaceDefault";
    }
}
