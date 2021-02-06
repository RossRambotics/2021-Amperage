package frc.robot.helper.DriveHandlingSetup;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveModes.*;
import frc.robot.subsystems.Drive;

// Override getter functions for base values to change handling characteristics of the robot
// Do not override getter functions for Calculated Values!!!

public class HandlingBase { // extend this class to create a unique set of handling characteristics
    private String m_tabName;
    protected String m_defaultDriveCommandName;

    protected double m_maxDriveOutput = 0.0; // the max power usable by any drive falcon

    protected double m_tankDeadZone = 0.15; // the deadzone in the joysticks for tankdrive
    protected double m_tankFineHandlingZone = 0.8; // the zone in the tank drive algorithm dedicated for precise driving
    protected double m_tankFineHandlingMaxVelocity = 0.07; // maximum relative velocity avaliable for tank fine handling

    protected double m_arcadeLowTurnZone = 0.5; // the percentage in the x values of the arcade
    protected double m_arcadeLowMaxTurn = 0.25; // percentage of turning wieght avaliable in the arcade low turn zone

    protected double m_talonTankDriveKp = 0.1; // Drive talon default PID kP
    protected double m_talonTankDriveKi = 0.1; // Drive talon default PID kI
    protected double m_talonTankDriveKd = 0.1; // Drive talon default PID KD

    private double m_tankFineHandlingCoefficent; // Calculated value
    private double m_tankHighPowerCoefficent; // Calculated value

    private double m_arcadeLowTurnCoefficent; // Calculated value
    private double m_arcadeHighTurnCoefficent;// Calculated value

    private NetworkTableEntry m_maxDriveOutputEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_deadZoneEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_fineHandlingZoneEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_fineHandlingMaxVelocityEntry; // Network Table Entry for Shuffleboard

    private NetworkTableEntry m_talonTankDriveKpEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_talonTankDriveKiEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_talonTankDriveKdEntry; // Network Table Entry for Shuffleboard

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

        if (handlingCalibrations.getComponents().size() == 0) { // if there are components in the list already do not
                                                                // run
            m_maxDriveOutputEntry = handlingCalibrations.add("Max Power", m_maxDriveOutput).getEntry();
            m_deadZoneEntry = handlingCalibrations.add("Joystick Deadzone", m_tankDeadZone).getEntry();
            m_fineHandlingZoneEntry = handlingCalibrations.add("Fine Handling Zone", m_tankFineHandlingZone).getEntry();
            m_fineHandlingMaxVelocityEntry = handlingCalibrations
                    .add("High Velocity Zone", m_tankFineHandlingMaxVelocity).getEntry();
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
        m_tankDeadZone = getTankDeadZone();
        m_tankFineHandlingZone = getTankFineHandlingZone();
        m_tankFineHandlingMaxVelocity = getTankFineHandlingMaxVelocity();

        m_arcadeLowMaxTurn = getArcadeLowMaxTurn();
        m_arcadeLowTurnZone = getArcadeLowTurnZone();

        m_talonTankDriveKp = getTalonTankDriveKp();
        m_talonTankDriveKi = getTalonTankDriveKi();
        m_talonTankDriveKd = getTalonTankDriveKd();

        m_defaultDriveCommandName = getDefaultDriveCommandName();
        m_tabName = getTabName();
    }

    private void setCalculatedMemberVariables() // sets and calculates member contants
    {
        m_tankFineHandlingCoefficent = m_tankFineHandlingMaxVelocity / (m_tankFineHandlingZone - m_tankDeadZone);
        m_tankHighPowerCoefficent = (Math.pow(1 - m_tankFineHandlingMaxVelocity, .333333333333333))
                / (1 - m_tankFineHandlingZone);

        m_arcadeLowTurnCoefficent = m_arcadeLowMaxTurn / m_arcadeLowTurnZone;
        m_arcadeHighTurnCoefficent = (1 - m_arcadeLowMaxTurn) / (1 - m_arcadeLowTurnZone);
    }

    public void refreshNetworkTablesValues() // refreshes values to the ones from network tables -> defaults must be
                                             // updated in code
    {
        m_maxDriveOutput = m_maxDriveOutputEntry.getDouble(m_maxDriveOutput);
        m_tankDeadZone = m_deadZoneEntry.getDouble(m_tankDeadZone);
        m_tankFineHandlingZone = m_fineHandlingZoneEntry.getDouble(m_tankFineHandlingZone);
        m_tankFineHandlingMaxVelocity = m_fineHandlingMaxVelocityEntry.getDouble(m_tankFineHandlingMaxVelocity);

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

    public double getTankDeadZone() {
        return 0.15;
    }

    public double getTankFineHandlingZone() {
        return 0.8;
    }

    public double getTankFineHandlingMaxVelocity() {
        return 0.7;
    }

    public double getArcadeLowMaxTurn() {
        return 0.25;
    }

    public double getArcadeLowTurnZone() {
        return 0.5;
    }

    public double getArcadeLowTurnCoefficent() {
        return m_arcadeLowTurnCoefficent;
    }

    public double getArcadeHighTurnCoefficent() {
        return m_arcadeHighTurnCoefficent;
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

    final public double getTankFineHandlingCoefficent() {
        return m_tankFineHandlingCoefficent;
    }

    final public double getTankHighPowerCoefficent() {
        return m_tankHighPowerCoefficent;
    }

}
