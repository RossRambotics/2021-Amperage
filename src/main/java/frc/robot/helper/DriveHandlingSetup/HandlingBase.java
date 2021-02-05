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

public class HandlingBase { // extend this class to create a unique set of handling characteristics
    private String m_tabName;
    protected String m_defaultDriveCommandName;

    protected double m_maxDriveOutput = 0.0;
    protected double m_deadZone = 0.15;
    protected double m_fineHandlingZone = 0.8;
    protected double m_fineHandlingMaxVelocity = 0.07;

    protected double m_talonTankDriveKp = 0.1;
    protected double m_talonTankDriveKi = 0.1;
    protected double m_talonTankDriveKd = 0.1;

    private double m_fineHandlingCoefficent;
    private double m_highPowerCoefficent;

    private NetworkTableEntry m_maxDriveOutputEntry;
    private NetworkTableEntry m_deadZoneEntry;
    private NetworkTableEntry m_fineHandlingZoneEntry;
    private NetworkTableEntry m_fineHandlingMaxVelocityEntry;

    private NetworkTableEntry m_talonTankDriveKpEntry;
    private NetworkTableEntry m_talonTankDriveKiEntry;
    private NetworkTableEntry m_talonTankDriveKdEntry;

    public HandlingBase() {
        setBaseMemberVariables();
        setCalculatedMemberVariables();
        createShuffleBoardTab();
    }

    private void createShuffleBoardTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("Sub.Handling " + m_tabName);

        ShuffleboardLayout talonCalibrations = tab.getLayout("Talon Calibration", BuiltInLayouts.kList).withSize(2, 5)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        ShuffleboardLayout handlingCalibrations = tab.getLayout("Handling Calibration", BuiltInLayouts.kList)
                .withSize(2, 5).withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        System.out.println(handlingCalibrations.getComponents());

        if (talonCalibrations.getComponents().size() == 0) { // if there are components in the list already do not run
            m_maxDriveOutputEntry = handlingCalibrations.add("Max Power", m_maxDriveOutput).getEntry();
            m_deadZoneEntry = handlingCalibrations.add("Joystick Deadzone", m_deadZone).getEntry();
            m_fineHandlingZoneEntry = handlingCalibrations.add("Fine Handling Zone", m_fineHandlingZone).getEntry();
            m_fineHandlingMaxVelocityEntry = handlingCalibrations.add("High Velocity Zone", m_fineHandlingMaxVelocity)
                    .getEntry();
        }

        if (talonCalibrations.getComponents().size() == 0) {// if there are already components in the list do not run
            m_talonTankDriveKpEntry = talonCalibrations.add("Drive Talon TankDrive Kp", m_talonTankDriveKp).getEntry();
            m_talonTankDriveKiEntry = talonCalibrations.add("Drive Talon TankDrive Ki", m_talonTankDriveKi).getEntry();
            m_talonTankDriveKdEntry = talonCalibrations.add("Drive Talon TankDrive Kd", m_talonTankDriveKd).getEntry();
        }
    }

    private void setBaseMemberVariables() // sets all of the base member variable values
    {
        m_maxDriveOutput = getMaxDriveOutput();
        m_deadZone = getDeadZone();
        m_fineHandlingZone = getFineHandlingZone();
        m_fineHandlingMaxVelocity = getfineHandlingMaxVelocity();

        m_talonTankDriveKp = getTalonTankDriveKp();
        m_talonTankDriveKi = getTalonTankDriveKi();
        m_talonTankDriveKd = getTalonTankDriveKd();

        m_defaultDriveCommandName = getDefaultDriveCommandName();

    }

    private void setCalculatedMemberVariables() // sets and calculates member contants
    {
        m_fineHandlingCoefficent = m_fineHandlingMaxVelocity / (m_fineHandlingZone - m_deadZone);
        m_highPowerCoefficent = (Math.pow(1 - m_fineHandlingMaxVelocity, .333333333333333)) / (1 - m_fineHandlingZone);
    }

    public void refreshNetworkTablesValues() // refreshes values to the ones from network tables -> defaults must be
                                             // updated in code
    {
        m_maxDriveOutput = m_maxDriveOutputEntry.getDouble(m_maxDriveOutput);
        m_deadZone = m_deadZoneEntry.getDouble(m_deadZone);
        m_fineHandlingZone = m_fineHandlingZoneEntry.getDouble(m_fineHandlingZone);
        m_fineHandlingMaxVelocity = m_fineHandlingMaxVelocityEntry.getDouble(m_fineHandlingMaxVelocity);

        m_talonTankDriveKp = m_talonTankDriveKpEntry.getDouble(m_talonTankDriveKp);
        m_talonTankDriveKi = m_talonTankDriveKiEntry.getDouble(m_talonTankDriveKi);
        m_talonTankDriveKd = m_talonTankDriveKdEntry.getDouble(m_talonTankDriveKd);
    }

    public Command getDefaultDriveCommand(Drive drive) {
        switch (m_defaultDriveCommandName) {
            case "TankDrive":
                System.out.println("TankDrive Running");
                return new RunTankDrive(drive);

            case "TankDriveHandBrake":
                System.out.println("TankDriveHandBrake Running");
                return new RunTankDriveHandBrake(drive);

            case "ArcadeDrive":
                System.out.println("ArcadeDrive Running");
                return new RunArcadeDrive(drive);

            default:
                System.out.println("TankDrive is Running by default!!!!");
                return new RunTankDrive(drive);
        }
    }

    public double getMaxDriveOutput() {
        return 0.0;
    }

    public double getDeadZone() {
        return 0.15;
    }

    public double getFineHandlingZone() {
        return 0.8;
    }

    public double getfineHandlingMaxVelocity() {
        return 0.7;
    }

    public double getTalonTankDriveKp() {
        return 0.1;
    }

    public double getTalonTankDriveKi() {
        return 0;
    }

    public double getTalonTankDriveKd() {
        return 0;
    }

    protected String getDefaultDriveCommandName() {
        return "None";
    }

    protected String getTabName() {
        return "default";
    }

    final public double getFineHandlingCoefficent() {
        return m_fineHandlingCoefficent;
    }

    final public double getHighPowerCoefficent() {
        return m_highPowerCoefficent;
    }

}
