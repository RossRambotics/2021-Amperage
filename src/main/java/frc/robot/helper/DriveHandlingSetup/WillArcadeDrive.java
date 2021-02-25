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
    public double getMaxVelocity() {
        return 7;
    }

    @Override
    public double getArcadeLowTurnCoefficent() {
        return 0.15;
    }

    @Override
    public double getArcadeLowTurnZone() {
        return 0.9;
    }
}