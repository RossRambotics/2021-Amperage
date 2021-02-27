package frc.robot.helper.DriveHandlingSetup;

import frc.robot.subsystems.Drive;

public class RyanTankDrive extends HandlingBase {

    @Override
    public double getMaxDriveOutputInitialValue() {
        return 0.5;
    }

    @Override
    public String getDefaultDriveCommandNameInitialValue() {
        return "TankDriveHandBrake";
    }

    @Override
    protected double getTankFineHandlingMaxVelocityInitialValue() {
        return 0.1;
    }

    @Override
    public String getTabName() {
        return "RWT";
    }

    @Override
    public double getMaxVelocityInitialValue() {
        return 7;
    }
}