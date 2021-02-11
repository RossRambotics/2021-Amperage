package frc.robot.helper.DriveHandlingSetup;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveModes.*;
import frc.robot.subsystems.Drive;

// Override getter functions for base values to change handling characteristics of the robot
// Do not override getter functions for Calculated Values!!!

public class HandlingBase { // extend this class to create a unique set of handling characteristics
    public Boolean m_refreshShuffleBoard; // wether or not to refresh the shuffleboard each peroidic loop

    private String m_tabName;
    protected String m_defaultDriveCommandName;

    protected Double m_maxDriveOutput = 0.0; // the max power usable by any drive falcon

    protected Double m_tankDeadZone = 0.15; // the deadzone in the joysticks for tankdrive
    protected Double m_tankFineHandlingZone = 0.8; // the zone in the tank drive algorithm dedicated for precise driving
    protected Double m_tankFineHandlingMaxVelocity = 0.07; // maximum relative velocity avaliable for tank fine handling

    protected Double m_arcadeLowTurnZone = 0.5; // the percentage in the x values of the arcade
    protected Double m_arcadeLowMaxTurn = 0.25; // percentage of turning wieght avaliable in the arcade low turn zone

    protected Double m_talonTankDriveKp = 0.1; // Drive talon default PID kP
    protected Double m_talonTankDriveKi = 0.1; // Drive talon default PID kI
    protected Double m_talonTankDriveKd = 0.1; // Drive talon default PID KD

    protected Double m_angleAdjustmentkP; // the kp for the angle adjustment PIDs
    protected Double m_angleAdjustmentkI; // the kp for the angle adjustment PIDs
    protected Double m_angleAdjustmentkD; // the kp for the angle adjustment PIDs

    protected Double m_straightVelocityControlkP; // the kP for velocity control in a straight line
    protected Double m_straightVelocityControlkI; // the kI for velocity control in a straight line
    protected Double m_straightVelocityControlkD; // the kD for velocity control in a straight line

    private Double m_tankFineHandlingCoefficent; // Calculated value
    private Double m_tankHighPowerCoefficent; // Calculated value

    private Double m_arcadeLowTurnCoefficent; // Calculated value
    private Double m_arcadeHighTurnCoefficent;// Calculated value

    private NetworkTableEntry m_maxDriveOutputEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_deadZoneEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_fineHandlingZoneEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_fineHandlingMaxVelocityEntry; // Network Table Entry for Shuffleboard

    private NetworkTableEntry m_arcadeLowTurnZoneEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_arcadeLowMaxTurnEntry; // Network Table Entry for Shuffleboard

    private NetworkTableEntry m_talonTankDriveKpEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_talonTankDriveKiEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_talonTankDriveKdEntry; // Network Table Entry for Shuffleboard

    private NetworkTableEntry m_angleAdjustmentKpEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_angleAdjustmentKiEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_angleAdjustmentKdEntry; // Network Table Entry for Shuffleboard

    private NetworkTableEntry m_striaghtVelocityControlkPEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_striaghtVelocityControlkIEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_striaghtVelocityControlkDEntry; // Network Table Entry for Shuffleboard

    public HandlingBase() {
        setBaseMemberVariables();
        setCalculatedMemberVariables();
        createShuffleBoardTab();
    }

    private void createShuffleBoardTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("Sub.Handling " + m_tabName);

        if (tab.getComponents().size() == 0) {// if the tab is not populated alredy
            m_maxDriveOutputEntry = tab.add("Max Power", m_maxDriveOutput).withWidget(BuiltInWidgets.kNumberBar)
                    .withSize(2, 1).withProperties(Map.of("min", -1, "max", 1)).withPosition(0, 0).getEntry();
            m_deadZoneEntry = tab.add("Tank Dead Zone", m_tankDeadZone).withWidget(BuiltInWidgets.kNumberBar)
                    .withSize(2, 1).withProperties(Map.of("min", -1, "max", 1)).withPosition(0, 1).getEntry();
            m_fineHandlingZoneEntry = tab.add("Tank Fine Handling Zone", m_tankFineHandlingZone)
                    .withWidget(BuiltInWidgets.kNumberBar).withSize(2, 1).withProperties(Map.of("min", -1, "max", 1))
                    .withPosition(0, 2).getEntry();
            m_maxDriveOutputEntry = tab.add("Tank Fine Handling Max Relative Velocity", m_tankFineHandlingMaxVelocity)
                    .withWidget(BuiltInWidgets.kNumberBar).withSize(2, 1).withProperties(Map.of("min", -1, "max", 1))
                    .withPosition(0, 3).getEntry();

            m_arcadeLowMaxTurnEntry = tab.add("Arcade Drive Low Turn Max Power", m_arcadeLowMaxTurn)
                    .withWidget(BuiltInWidgets.kNumberBar).withSize(2, 1).withProperties(Map.of("min", -1, "max", 1))
                    .withPosition(2, 0).getEntry();
            m_arcadeLowMaxTurnEntry = tab.add("Arcade Drive Low Turn Zone", m_arcadeLowTurnZone)
                    .withWidget(BuiltInWidgets.kNumberBar).withSize(2, 1).withProperties(Map.of("min", -1, "max", 1))
                    .withPosition(2, 1).getEntry();

            m_maxDriveOutputEntry = tab.add("Talon Tank Kp", m_talonTankDriveKp).withWidget(BuiltInWidgets.kTextView)
                    .withSize(1, 1).withPosition(4, 0).getEntry();
            m_maxDriveOutputEntry = tab.add("Talon Tank Ki", m_talonTankDriveKi).withWidget(BuiltInWidgets.kTextView)
                    .withSize(1, 1).withPosition(4, 1).getEntry();
            m_maxDriveOutputEntry = tab.add("Talon Tank Kd", m_talonTankDriveKd).withWidget(BuiltInWidgets.kTextView)
                    .withSize(1, 1).withPosition(4, 2).getEntry();

            m_angleAdjustmentKpEntry = tab.add("Angle Adjustment Kp", m_angleAdjustmentkP)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(5, 0).getEntry();
            m_angleAdjustmentKiEntry = tab.add("Angle Adjustment Ki", m_angleAdjustmentkI)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(5, 1).getEntry();
            m_angleAdjustmentKdEntry = tab.add("Angle Adjustment Kd", m_angleAdjustmentkD)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(5, 2).getEntry();

            m_striaghtVelocityControlkPEntry = tab.add("Straight Line Kp", m_straightVelocityControlkP)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(5, 0).getEntry();
            m_striaghtVelocityControlkIEntry = tab.add("Straight Line Ki", m_straightVelocityControlkI)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(5, 1).getEntry();
            m_striaghtVelocityControlkDEntry = tab.add("Straight Line Kd", m_straightVelocityControlkD)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(5, 2).getEntry();
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

        m_angleAdjustmentkP = getAngleAdjustmentkP();
        m_angleAdjustmentkI = getAngleAdjustmentkI();
        m_angleAdjustmentkP = getAngleAdjustmentkD();

        m_straightVelocityControlkP = getStraightVelocityControlkP();
        m_straightVelocityControlkI = getStraightVelocityControlkI();
        m_straightVelocityControlkD = getStraightVelocityControlkD();

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
        if (getRefreshShuffleboard() == false) { // dont run if not needed
            return;
        }

        m_maxDriveOutput = m_maxDriveOutputEntry.getDouble(m_maxDriveOutput);
        m_tankDeadZone = m_deadZoneEntry.getDouble(m_tankDeadZone);
        m_tankFineHandlingZone = m_fineHandlingZoneEntry.getDouble(m_tankFineHandlingZone);
        m_tankFineHandlingMaxVelocity = m_fineHandlingMaxVelocityEntry.getDouble(m_tankFineHandlingMaxVelocity);

        m_arcadeLowMaxTurn = m_arcadeLowMaxTurnEntry.getDouble(m_arcadeLowMaxTurn);
        m_arcadeLowTurnZone = m_arcadeLowTurnZoneEntry.getDouble(m_arcadeLowTurnZone);

        m_talonTankDriveKp = m_talonTankDriveKpEntry.getDouble(m_talonTankDriveKp);
        m_talonTankDriveKi = m_talonTankDriveKiEntry.getDouble(m_talonTankDriveKi);
        m_talonTankDriveKd = m_talonTankDriveKdEntry.getDouble(m_talonTankDriveKd);

        m_angleAdjustmentkP = m_angleAdjustmentKpEntry.getDouble(m_angleAdjustmentkP);
        m_angleAdjustmentkI = m_angleAdjustmentKiEntry.getDouble(m_angleAdjustmentkI);
        m_angleAdjustmentkD = m_angleAdjustmentKdEntry.getDouble(m_angleAdjustmentkD);

        m_straightVelocityControlkP = m_striaghtVelocityControlkPEntry.getDouble(m_straightVelocityControlkP);
        m_straightVelocityControlkI = m_striaghtVelocityControlkIEntry.getDouble(m_straightVelocityControlkI);
        m_straightVelocityControlkD = m_striaghtVelocityControlkDEntry.getDouble(m_straightVelocityControlkD);

        setCalculatedMemberVariables(); // must redo the math :(
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

    protected Boolean getRefreshShuffleboardInitialValue() {
        return true;
    }

    protected double getMaxDriveOutputInitialValue() {
        return 0.0;
    }

    protected double getTankDeadZoneInitialValue() {
        return 0.15;
    }

    protected double getTankFineHandlingZoneInitialValue() {
        return 0.8;
    }

    protected double getTankFineHandlingMaxVelocityInitialValue() {
        return 0.2;
    }

    protected double getArcadeLowMaxTurnInitialValue() {
        return 0.25;
    }

    protected double getArcadeLowTurnZoneInitialValue() {
        return 0.5;
    }

    protected double getTalonTankDriveKpInitialValue() {
        return 0.1;
    }

    protected double getTalonTankDriveKiInitialValue() {
        return 0;
    }

    protected double getTalonTankDriveKdInitialValue() {
        return 0;
    }

    protected String getDefaultDriveCommandNameInitialValue() {
        return "None";
    }

    protected double getAngleAdjustmentkPInitialValue() {
        return 0.006;
    }

    protected double getAngleAdjustmentkIInitialValue() {
        return 0.02;
    }

    protected double getAngleAdjustmentkDInitialValue() {
        return 0.0013;
    }

    protected double getStraightVelocityControlkPInitailValue() {
        return 0.0;
    }

    protected double getStraightVelocityControlkIInitailValue() {
        return 0.0;
    }

    protected double getStraightVelocityControlkDInitailValue() {
        return 0.0;
    }

    public boolean getRefreshShuffleboard() {
        if (m_refreshShuffleBoard != null) {
            return m_refreshShuffleBoard;
        }

        return getRefreshShuffleboardInitialValue();
    }

    public double getMaxDriveOutput() {
        if (m_maxDriveOutput != null) {
            return m_maxDriveOutput;
        }

        return getMaxDriveOutputInitialValue();
    }

    public double getTankDeadZone() {
        if (m_tankDeadZone != null) {
            return m_tankDeadZone;
        }

        return getTankDeadZoneInitialValue();
    }

    public double getTankFineHandlingZone() {
        if (m_tankFineHandlingZone != null) {
            return m_tankFineHandlingZone;
        }

        return getTankFineHandlingZoneInitialValue();
    }

    public double getTankFineHandlingMaxVelocity() {
        if (m_tankFineHandlingMaxVelocity != null) {
            return m_tankFineHandlingMaxVelocity;
        }

        return getTankFineHandlingMaxVelocityInitialValue();
    }

    public double getArcadeLowMaxTurn() {
        if (m_arcadeLowMaxTurn != null) {
            return m_arcadeLowMaxTurn;
        }

        return getArcadeLowMaxTurnInitialValue();
    }

    public double getArcadeLowTurnZone() {
        if (m_arcadeLowTurnZone != null) {
            return m_arcadeLowTurnZone;
        }

        return getArcadeLowTurnZoneInitialValue();
    }

    public double getArcadeLowTurnCoefficent() {
        return m_arcadeLowTurnCoefficent;
    }

    public double getArcadeHighTurnCoefficent() {
        return m_arcadeHighTurnCoefficent;
    }

    public double getTalonTankDriveKp() {
        if (m_talonTankDriveKp != null) {
            return m_talonTankDriveKp;
        }

        return getTalonTankDriveKp();
    }

    public double getTalonTankDriveKi() {
        if (m_talonTankDriveKi != null) {
            return m_talonTankDriveKi;
        }

        return getTalonTankDriveKi();
    }

    public double getTalonTankDriveKd() {
        if (m_talonTankDriveKd != null) {
            return m_talonTankDriveKd;
        }

        return getTalonTankDriveKd();
    }

    protected String getDefaultDriveCommandName() {
        if (m_defaultDriveCommandName != null) {
            return m_defaultDriveCommandName;
        }

        return getDefaultDriveCommandNameInitialValue();
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

    public double getAngleAdjustmentkP() {
        if (m_angleAdjustmentkP != null) {
            return m_angleAdjustmentkP;
        }

        return getAngleAdjustmentkPInitialValue();
    }

    public double getAngleAdjustmentkI() {
        if (m_angleAdjustmentkI != null) {
            return m_angleAdjustmentkI;
        }

        return getAngleAdjustmentkIInitialValue();
    }

    public double getAngleAdjustmentkD() {
        if (m_angleAdjustmentkD != null) {
            return m_angleAdjustmentkD;
        }

        return getAngleAdjustmentkDInitialValue();
    }

    public double getStraightVelocityControlkP() {
        if (m_straightVelocityControlkP != null) {
            return m_straightVelocityControlkP;
        }

        return getStraightVelocityControlkPInitailValue();
    }

    public double getStraightVelocityControlkI() {
        if (m_straightVelocityControlkI != null) {
            return m_straightVelocityControlkI;
        }

        return getStraightVelocityControlkIInitailValue();
    }

    public double getStraightVelocityControlkD() {
        if (m_straightVelocityControlkD != null) {
            return m_straightVelocityControlkD;
        }

        return getStraightVelocityControlkDInitailValue();
    }
}
