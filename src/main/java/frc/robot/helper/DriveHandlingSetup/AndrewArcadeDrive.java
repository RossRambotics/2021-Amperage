package frc.robot.helper.DriveHandlingSetup;

import frc.robot.subsystems.Drive;

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
    protected double getMaxVelocityInitialValue() {
        return 7;
    }

    @Override
    public String getTabName() {
        return "AMA";
    }

    @Override
    public double getArcadeLowMaxTurnInitialValue() {
        return .15;
    }

    @Override
    public double getArcadeLowTurnZoneInitialValue() {
        return 0.9;
    }
}