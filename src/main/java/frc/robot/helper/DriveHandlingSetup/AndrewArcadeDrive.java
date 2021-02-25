package frc.robot.helper.DriveHandlingSetup;

public class AndrewArcadeDrive extends HandlingBase {
    @Override
    public double getMaxDriveOutputInitialValue() {
        return 0.5;
    }

    @Override
    public String getDefaultDriveCommandNameInitialValue() {
        return "ArcadeDrive";
    }

    @Override
    public double getMaxVelocity() {
        return 7;
    }

    @Override
    public String getTabName() {
        return "AMA";
    }

    @Override
    public double getArcadeLowMaxTurn() {
        return .15;
    }

    @Override
    public double getArcadeLowTurnZone() {
        return 0.9;
    }
}