package frc.robot.helper.DriveHandlingSetup;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.RunTankDrive;
import frc.robot.commands.RunTankDriveHandBrake;
import frc.robot.subsystems.Drive;

public class HandlingBase {
    private String m_name = "Default";
    private String m_defaultDriveCommandName = "None";

    protected Double m_maxDriveOutput = 0.5;
    protected Double m_deadZone = 0.15;
    protected Double m_fineHandlingZone = 0.8;
    protected Double m_fineHandlingMaxVeloctiy = 0.07;

    private NetworkTableEntry m_maxDriveOutputEntry;
    private NetworkTableEntry m_deadZoneEntry;
    private NetworkTableEntry m_fineHandlingZoneEntry;
    private NetworkTableEntry m_fineHandlingMaxVeloctiyEntry;

    public HandlingBase() {
        createShuffleBoardTab();
    }

    private void createShuffleBoardTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("Sub.Handling " + m_name);

        ShuffleboardLayout talonCalibrations = tab.getLayout("Talon Calibration", BuiltInLayouts.kList).withSize(2, 5)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        ShuffleboardLayout handlingCalibrations = tab.getLayout("Handling Calibration", BuiltInLayouts.kList)
                .withSize(2, 5).withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        m_maxDriveOutputEntry = handlingCalibrations.add("Max Power", m_maxDriveOutput).getEntry();
        m_deadZoneEntry = handlingCalibrations.add("Max Power", m_deadZoneEntry).getEntry();
        m_fineHandlingZoneEntry = handlingCalibrations.add("Max Power", m_fineHandlingZone).getEntry();
        m_fineHandlingMaxVeloctiyEntry = handlingCalibrations.add("Max Power", m_fineHandlingMaxVeloctiyEntry)
                .getEntry();

        // m_maxDriveOutputEntry = talonCalibrations.add("MAX POWER!",
        // 0.5).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(4, 1).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    }

    public Command getDefaultDriveCommand(Drive drive) {
        switch (m_defaultDriveCommandName) {
            case "TankDrive":
                return new RunTankDrive(drive);

            case "TankDriveHandBrake":
                return new RunTankDriveHandBrake(drive);

            default:
                return new RunTankDrive(drive);
        }
    }

    public Double getMaxDriveOutput() {
        return m_maxDriveOutput;
    }

    public Double getDeadZone() {
        return m_deadZone;
    }

    public Double getFineHandlingZone() {
        return m_fineHandlingZone;
    }

    public Double getfineHandlingMaxVelocity() {
        return m_fineHandlingMaxVeloctiy;
    }

}
