package frc.robot.helper.DriveHandlingSetup;

public class DefaultHardSurfaceArcadeDrive extends HandlingBase {
    @Override
    public double getMaxDriveOutput() {
        return 0.5;
    }

    @Override
    public String getDefaultDriveCommandName() {
        return "ArcadeDrive";
    }

    @Override
    public String getTabName() {
        return "HardSurfaceDefault";
    }
}