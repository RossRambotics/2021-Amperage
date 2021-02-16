package frc.robot.helper.DriveHandlingSetup;

public class DefaultHardSurfaceArcadeDrive extends HandlingBase {
    @Override
    public double getMaxDriveOutputInitialValue() {
        return 0.5;
    }

    @Override
    public String getDefaultDriveCommandNameInitialValue() {
        return "ArcadeDrive";
    }

    @Override
    public String getTabName() {
        return "HardSurfaceDefaultArcade";
    }
}