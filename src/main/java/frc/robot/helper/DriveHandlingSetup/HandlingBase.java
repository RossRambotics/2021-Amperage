package frc.robot.helper.DriveHandlingSetup;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveModes.*;
import frc.robot.subsystems.Drive;

public class HandlingBase {
    private String m_name = "Default";
    private String m_defaultDriveCommandName = "None";

    protected Double m_maxDriveOutput = 0.0;
    protected Double m_deadZone = 0.15;
    protected Double m_fineHandlingZone = 0.8;
    protected Double m_fineHandlingMaxVeloctiy = 0.07;

    protected Double m_talonTankDriveKp = 0.1;
    protected Double m_talonTankDriveKi = 0.1;
    protected Double m_talonTankDriveKd = 0.1;

    private NetworkTableEntry m_maxDriveOutputEntry;
    private NetworkTableEntry m_deadZoneEntry;
    private NetworkTableEntry m_fineHandlingZoneEntry;
    private NetworkTableEntry m_fineHandlingMaxVeloctiyEntry;

    private NetworkTableEntry m_talonTankDriveKpEntry;
    private NetworkTableEntry m_talonTankDriveKiEntry;
    private NetworkTableEntry m_talonTankDriveKdEntry;

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
        m_deadZoneEntry = handlingCalibrations.add("Joystick Deadzone", m_deadZone).getEntry();
        m_fineHandlingZoneEntry = handlingCalibrations.add("Fine Handling Zone", m_fineHandlingZone).getEntry();
        m_fineHandlingMaxVeloctiyEntry = handlingCalibrations.add("High Velocity Zone", m_fineHandlingMaxVeloctiy)
                .getEntry();

        m_talonTankDriveKpEntry = talonCalibrations.add("Drive Talon TankDrive Kp", m_talonTankDriveKp).getEntry();
        m_talonTankDriveKiEntry = talonCalibrations.add("Drive Talon TankDrive Ki", m_talonTankDriveKi).getEntry();
        m_talonTankDriveKdEntry = talonCalibrations.add("Drive Talon TankDrive Kd", m_talonTankDriveKd).getEntry();
    }

    public void refreshNetworkTablesValues() // refreshes values to the ones from network tables -> defaults must be
                                             // updated in code
    {
        m_maxDriveOutput = m_maxDriveOutputEntry.getDouble(m_maxDriveOutput);
        m_deadZone = m_deadZoneEntry.getDouble(m_deadZone);
        m_fineHandlingZone = m_fineHandlingZoneEntry.getDouble(m_fineHandlingZone);
        m_fineHandlingMaxVeloctiy = m_fineHandlingMaxVeloctiyEntry.getDouble(m_fineHandlingMaxVeloctiy);

        m_talonTankDriveKp = m_talonTankDriveKpEntry.getDouble(m_talonTankDriveKp);
        m_talonTankDriveKi = m_talonTankDriveKiEntry.getDouble(m_talonTankDriveKi);
        m_talonTankDriveKd = m_talonTankDriveKdEntry.getDouble(m_talonTankDriveKd);
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

    public Double getTalonTankDriveKp() {
        return m_talonTankDriveKp;
    }

    public Double getTalonTankDriveKi() {
        return m_talonTankDriveKi;
    }

    public Double getTalonTankDriveKd() {
        return m_talonTankDriveKd;
    }

}
