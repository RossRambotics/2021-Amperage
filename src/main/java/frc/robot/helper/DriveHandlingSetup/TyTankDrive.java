package frc.robot.helper.DriveHandlingSetup;

public class TyTankDrive extends HandlingBase {
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
        return 0.15;
    }

    @Override
    public String getTabName() {
        return "TAT";
    }

    @Override
    public double getMaxVelocityInitialValue() {
        return 13;
    }
}