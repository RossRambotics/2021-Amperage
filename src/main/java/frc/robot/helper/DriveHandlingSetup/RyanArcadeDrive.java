package frc.robot.helper.DriveHandlingSetup;

public class RyanArcadeDrive extends HandlingBase {
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
        return "RAD";
    }

    @Override
    public double getMaxVelocityInitialValue() {
        return 7;
    }
}