package frc.robot.helper.DriveHandlingSetup;

public class TyArcadeDrive extends HandlingBase {
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
        return "TAA";
    }
}