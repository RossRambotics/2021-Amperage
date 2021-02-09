package frc.robot.helper.DriveHandlingSetup;

public class DefaultHardSurfaceHandling extends HandlingBase {
    @Override
    public double getMaxDriveOutput() {
        return 0.5;
    }

    @Override
    public String getDefaultDriveCommandName() {
        return "TankDriveHandBrake";
    }

    @Override
    public String getTabName() {
        return "HardSurfaceDefault";
    }
}
