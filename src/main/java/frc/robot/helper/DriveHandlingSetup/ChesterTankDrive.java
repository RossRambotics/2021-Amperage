package frc.robot.helper.DriveHandlingSetup;

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
    public double getMaxVelocityInitialValue() {
        return 25;
    }

    @Override
    public String getTabName() {
        return "CPT";
    }
}