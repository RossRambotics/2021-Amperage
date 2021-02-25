package frc.robot.helper.DriveHandlingSetup;

public class WillArcadeDrive extends HandlingBase {
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
        return "WLA";
    }

    @Override
    public double getMaxVelocityInitialValue() {
        return 7;
    }

    @Override
    public double getArcadeLowMaxTurnInitialValue() {
        return 0.15;
    }

    @Override
    public double getArcadeLowTurnZoneInitialValue() {
        return 0.9;
    }
}