package frc.robot.helper.DriveHandlingSetup;

public class TyTankDrive extends HandlingBase {
    @Override
    public double getMaxDriveOutputInitialValue() {
        return 0.5;
    }

    @Override
    public double getPowerCoefficentInitialValue() {
        return .5;
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
    protected double getRadialTurnCoefficentInitialValue() {
        return 0.1;
    }
}