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
    public double getPowerCoefficentInitialValue() {
        return .5;
    }

    @Override
    public String getTabName() {
        return "CPT";
    }

    @Override
    protected double getRadialTurnCoefficentInitialValue() {
        return 0.36;
    }

    @Override
    protected double getTankFineHandlingMaxVelocityInitialValue() {
        // TODO Auto-generated method stub
        return 0.2;
    }

    @Override
    protected double getTankFineHandlingZoneInitialValue() {
        return 0.99;
    }
}