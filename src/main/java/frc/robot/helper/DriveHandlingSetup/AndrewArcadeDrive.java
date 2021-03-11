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
    protected double getPowerCoefficentInitialValue() {
        return .5;
    }

    @Override
    public String getTabName() {
        return "AMA";
    }

    @Override
    public double getArcadeLowMaxTurnInitialValue() {
        return 0.7;
    }

    @Override
    public double getArcadeLowTurnZoneInitialValue() {
        return 0.9;
    }

    @Override
    public double getArcadeHighMaxTurnCoefficentInitalValue() {
        return 1;
    }

    @Override
    public double getRadialTurnCoefficentInitialValue() {
        return .36;
    }
}