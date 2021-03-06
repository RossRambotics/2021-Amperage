package frc.robot.helper.DriveHandlingSetup;

import frc.robot.subsystems.Drive;

public class ChesterTankDrive extends HandlingBase {

    @Override
    public double getMaxDriveOutputInitialValue() {
        return 0.5;
    }

    @Override
    public String getDefaultDriveCommandNameInitialValue() {
        return "TankDriveHandBrake";
    }

    @Override
    public double getPowerCoefficentInitialValue() {
        return 25;
    }

    @Override
    public String getTabName() {
        return "CPT";
    }

    @Override
    protected double getRadialTurnCoefficentInitalValue() {
        return 0.1;
    }
}