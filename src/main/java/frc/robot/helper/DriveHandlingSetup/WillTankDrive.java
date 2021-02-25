package frc.robot.helper.DriveHandlingSetup;

public class WillTankDrive extends HandlingBase {
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
        return "WLT";
    }

    @Override
    public double getMaxVelocity() {
        return 7;
    }
}