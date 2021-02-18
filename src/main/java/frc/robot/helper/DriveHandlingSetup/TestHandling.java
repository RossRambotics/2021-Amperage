package frc.robot.helper.DriveHandlingSetup;

public class TestHandling extends HandlingBase {
    @Override
    public double getMaxDriveOutputInitialValue() {
        return 0.5;
    }

    @Override
    public String getDefaultDriveCommandNameInitialValue() {
        return "TankDriveHandBrake";
    }

    @Override
    public double getTankFineHandlingMaxVelocityInitialValue() {
        return 0.2;
    }

    @Override
    public String getTabName() {
        return "TestHandling";
    }
}
