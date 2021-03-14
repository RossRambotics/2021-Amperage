package frc.robot.helper.DriveHandlingSetup;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
// Only override functions ending in InitialValue!

public class HandlingBase { // extend this class to create a unique set of handling characteristics
    public Boolean m_refreshShuffleBoard; // wether or not to refresh the shuffleboard each peroidic loop

    private String m_tabName;
    private String m_automationTabName = "Automation Constants"; // the name for the automation tab

    protected String m_defaultDriveCommandName;

    protected Double m_maxDriveOutput; // the max power usable by any drive falcon

    protected Double m_tankDeadZone; // the deadzone in the joysticks for tankdrive
    protected Double m_tankFineHandlingZone; // the zone in the tank drive algorithm dedicated for precise driving
    protected Double m_tankFineHandlingMaxVelocity; // maximum relative velocity avaliable for tank fine handling
    protected Double m_powerCoefficent; // in MPS
    protected Double m_radialTurnCoefficent; // the percent of velocity to provide to the inner wheel during a radial
                                             // turn

    protected Double m_arcadeLowTurnZone; // the percentage in the x values of the arcade
    protected Double m_arcadeLowMaxTurn; // percentage of turning wieght avaliable in the arcade low turn zone
    protected Double m_arcadeHighMaxTurnCoefficent; // maximum turn relative to the high max turn;
                                                    // 1 = full turn is max; 0 = low turn is max;

    protected Double m_talonTankDriveKp; // Drive talon default PID kP
    protected Double m_talonTankDriveKi; // Drive talon default PID kI
    protected Double m_talonTankDriveKd; // Drive talon default PID KD

    protected Double m_angleAdjustmentkP; // the kp for the angle adjustment PIDs
    protected Double m_angleAdjustmentkI; // the ki for the angle adjustment PIDs
    protected Double m_angleAdjustmentkD; // the kd for the angle adjustment PIDs

    protected Double m_targettingTurnKP; // the kp for the targetting turn
    protected Double m_targettingTurnKI; // the ki for the targetting turn
    protected Double m_targettingTurnKD; // the kd for the targetting turn

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
    private NetworkTableEntry m_powerCoefficentEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_radialTurnCoefficentEntry; // Network Table Entry for Shuffleboard

    private NetworkTableEntry m_arcadeLowTurnZoneEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_arcadeLowMaxTurnEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_arcadeHighMaxTurnCoefficentEntry; // Network Table Entry for Shuffleboard

    private NetworkTableEntry m_talonTankDriveKpEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_talonTankDriveKiEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_talonTankDriveKdEntry; // Network Table Entry for Shuffleboard

    private NetworkTableEntry m_angleAdjustmentKpEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_angleAdjustmentKiEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_angleAdjustmentKdEntry; // Network Table Entry for Shuffleboard

    private NetworkTableEntry m_straightVelocityControlkPEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_straightVelocityControlkIEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_straightVelocityControlkDEntry; // Network Table Entry for Shuffleboard

    private NetworkTableEntry m_targettingTurnKPEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_targettingTurnKIEntry; // Network Table Entry for Shuffleboard
    private NetworkTableEntry m_targettingTurnKDEntry; // Network Table Entry for Shuffleboard

    public HandlingBase() {
        setBaseMemberVariables();
        setCalculatedMemberVariables();
        createShuffleBoardTab();
    }

    private void createShuffleBoardTab() {
        ShuffleboardTab driverTab = Shuffleboard.getTab(m_tabName);
        ShuffleboardTab automationTab = Shuffleboard.getTab(m_automationTabName);

        if (driverTab.getComponents().size() == 0) {// if the tab is not populated alredy
            m_maxDriveOutputEntry = driverTab.add("Max Power", m_maxDriveOutput)
                    .withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1).withProperties(Map.of("min", 0, "max", 1))
                    .withPosition(0, 0).getEntry();
            m_deadZoneEntry = driverTab.add("Tank Dead Zone", m_tankDeadZone).withWidget(BuiltInWidgets.kNumberSlider)
                    .withSize(2, 1).withProperties(Map.of("min", 0, "max", 1)).withPosition(0, 1).getEntry();
            m_fineHandlingZoneEntry = driverTab.add("Tank Fine Handling Zone", m_tankFineHandlingZone)
                    .withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1)
                    .withProperties(Map.of("min", 0, "max", 0.99)).withPosition(0, 2).getEntry();
            m_fineHandlingMaxVelocityEntry = driverTab
                    .add("Tank Fine Handling Max Relative Velocity", m_tankFineHandlingMaxVelocity)
                    .withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1).withProperties(Map.of("min", 0, "max", 1))
                    .withPosition(0, 3).getEntry();
            m_powerCoefficentEntry = driverTab.add("Max Velocity", m_powerCoefficent)
                    .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).withSize(1, 1)
                    .withPosition(5, 3).getEntry();
            m_radialTurnCoefficentEntry = driverTab.add("Radial Turn Coefficent", m_radialTurnCoefficent)
                    .withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1).withProperties(Map.of("min", 0, "max", 1))
                    .withPosition(2, 3).getEntry();

            m_arcadeLowMaxTurnEntry = driverTab.add("Arcade Drive Low Turn Max Power", m_arcadeLowMaxTurn)
                    .withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1).withProperties(Map.of("min", 0, "max", 1))
                    .withPosition(2, 0).getEntry();
            m_arcadeLowTurnZoneEntry = driverTab.add("Arcade Drive Low Turn Zone", m_arcadeLowTurnZone)
                    .withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1)
                    .withProperties(Map.of("min", 0, "max", 0.99)).withPosition(2, 1).getEntry();
            m_arcadeHighMaxTurnCoefficentEntry = driverTab
                    .add("Arcade High Turn Coefficent", m_arcadeHighMaxTurnCoefficent)
                    .withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1).withProperties(Map.of("min", 0, "max", 1))
                    .withPosition(2, 2).getEntry();

            m_talonTankDriveKpEntry = driverTab.add("Talon Tank Kp", m_talonTankDriveKp)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(4, 0).getEntry();
            m_talonTankDriveKiEntry = driverTab.add("Talon Tank Ki", m_talonTankDriveKi)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(4, 1).getEntry();
            m_talonTankDriveKdEntry = driverTab.add("Talon Tank Kd", m_talonTankDriveKd)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(4, 2).getEntry();
        }

        if (automationTab.getComponents().size() == 0) { // makes sure the tab is not currently populated
            m_angleAdjustmentKpEntry = automationTab.add("Angle Adjustment Kp", m_angleAdjustmentkP)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(5, 0).getEntry();
            m_angleAdjustmentKiEntry = automationTab.add("Angle Adjustment Ki", m_angleAdjustmentkI)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(5, 1).getEntry();
            m_angleAdjustmentKdEntry = automationTab.add("Angle Adjustment Kd", m_angleAdjustmentkD)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(5, 2).getEntry();

            m_straightVelocityControlkPEntry = automationTab.add("Straight Line Kp", m_straightVelocityControlkP)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(6, 0).getEntry();
            m_straightVelocityControlkIEntry = automationTab.add("Straight Line Ki", m_straightVelocityControlkI)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(6, 1).getEntry();
            m_straightVelocityControlkDEntry = automationTab.add("Straight Line Kd", m_straightVelocityControlkD)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(6, 2).getEntry();

            m_targettingTurnKDEntry = automationTab.add("Targetting Turn KD", m_targettingTurnKD).withPosition(7, 2)
                    .withSize(1, 1).getEntry();
            m_targettingTurnKIEntry = automationTab.add("Targetting Turn KI", m_targettingTurnKI).withPosition(7, 1)
                    .withSize(1, 1).getEntry();
            m_targettingTurnKPEntry = automationTab.add("Targetting Turn KP", m_targettingTurnKP).withPosition(7, 0)
                    .withSize(1, 1).getEntry();
        } else {
            NetworkTable automationTable = NetworkTableInstance.getDefault().getTable("Shuffleboard")
                    .getSubTable(m_automationTabName);

            m_angleAdjustmentKpEntry = automationTable.getEntry("Angle Adjustment Kp");
            m_angleAdjustmentKiEntry = automationTable.getEntry("Angle Adjustment Ki");
            m_angleAdjustmentKdEntry = automationTable.getEntry("Angle Adjustment Kd");

            m_angleAdjustmentKpEntry.setDouble(getAngleAdjustmentkP());
            m_angleAdjustmentKiEntry.setDouble(getAngleAdjustmentkI());
            m_angleAdjustmentKdEntry.setDouble(getAngleAdjustmentkD());

            m_straightVelocityControlkPEntry = automationTable.getEntry("Straight Line Kp");
            m_straightVelocityControlkIEntry = automationTable.getEntry("Straight Line Kp");
            m_straightVelocityControlkDEntry = automationTable.getEntry("Straight Line Ki");

            m_straightVelocityControlkPEntry.setDouble(getStraightVelocityControlkP());
            m_straightVelocityControlkIEntry.setDouble(getStraightVelocityControlkP());
            m_straightVelocityControlkDEntry.setDouble(getStraightVelocityControlkP());

            m_targettingTurnKPEntry = automationTable.getEntry("Targetting Turn Kp");
            m_targettingTurnKIEntry = automationTable.getEntry("Targetting Turn KI");
            m_targettingTurnKDEntry = automationTable.getEntry("Targetting Turn KD");

            m_targettingTurnKPEntry.setDouble(getTargettingTurnKP());
            m_targettingTurnKIEntry.setDouble(getTargettingTurnKI());
            m_targettingTurnKDEntry.setDouble(getTargettingTurnKD());
        }

    }

    private void setBaseMemberVariables() // sets all of the base member variable values
    {
        m_maxDriveOutput = getMaxDriveOutput();
        m_tankDeadZone = getTankDeadZone();
        m_tankFineHandlingZone = getTankFineHandlingZone();
        m_tankFineHandlingMaxVelocity = getTankFineHandlingMaxVelocity();
        m_powerCoefficent = getPowerCoefficent();
        m_radialTurnCoefficent = getRadialTurnCoefficent();

        m_arcadeLowMaxTurn = getArcadeLowMaxTurn();
        m_arcadeLowTurnZone = getArcadeLowTurnZone();
        m_arcadeHighMaxTurnCoefficent = getArcadeHighMaxTurnCoefficent();

        m_talonTankDriveKp = getTalonTankDriveKp();
        m_talonTankDriveKi = getTalonTankDriveKi();
        m_talonTankDriveKd = getTalonTankDriveKd();
        // m_configureTalons = getUpdateTalonConfig();

        m_angleAdjustmentkP = getAngleAdjustmentkP();
        m_angleAdjustmentkI = getAngleAdjustmentkI();
        m_angleAdjustmentkD = getAngleAdjustmentkD();

        m_straightVelocityControlkP = getStraightVelocityControlkP();
        m_straightVelocityControlkI = getStraightVelocityControlkI();
        m_straightVelocityControlkD = getStraightVelocityControlkD();

        m_targettingTurnKP = getTargettingTurnKP();
        m_targettingTurnKI = getTargettingTurnKI();
        m_targettingTurnKD = getTargettingTurnKD();

        m_defaultDriveCommandName = getDefaultDriveCommandName();
        m_tabName = getTabName();
    }

    private void setCalculatedMemberVariables() // sets and calculates member contants
    {
        m_tankFineHandlingCoefficent = m_tankFineHandlingMaxVelocity / (m_tankFineHandlingZone - m_tankDeadZone);
        m_tankHighPowerCoefficent = (Math.pow(1 - m_tankFineHandlingMaxVelocity, .333333333333333))
                / (1 - m_tankFineHandlingZone);

        m_arcadeLowTurnCoefficent = m_arcadeLowMaxTurn / m_arcadeLowTurnZone;
        m_arcadeHighTurnCoefficent = m_arcadeHighMaxTurnCoefficent * (1 - m_arcadeLowMaxTurn)
                / (1 - m_arcadeLowTurnZone);
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
        m_powerCoefficent = m_powerCoefficentEntry.getDouble(20);
        m_radialTurnCoefficent = m_radialTurnCoefficentEntry.getDouble(m_radialTurnCoefficent);

        m_arcadeLowMaxTurn = m_arcadeLowMaxTurnEntry.getDouble(m_arcadeLowMaxTurn);
        m_arcadeLowTurnZone = m_arcadeLowTurnZoneEntry.getDouble(m_arcadeLowTurnZone);
        m_arcadeHighMaxTurnCoefficent = m_arcadeHighMaxTurnCoefficentEntry.getDouble(m_arcadeHighMaxTurnCoefficent);

        m_talonTankDriveKp = m_talonTankDriveKpEntry.getDouble(m_talonTankDriveKp);
        m_talonTankDriveKi = m_talonTankDriveKiEntry.getDouble(m_talonTankDriveKi);
        m_talonTankDriveKd = m_talonTankDriveKdEntry.getDouble(m_talonTankDriveKd);

        m_angleAdjustmentkP = m_angleAdjustmentKpEntry.getDouble(m_angleAdjustmentkP);
        m_angleAdjustmentkI = m_angleAdjustmentKiEntry.getDouble(m_angleAdjustmentkI);
        m_angleAdjustmentkD = m_angleAdjustmentKdEntry.getDouble(m_angleAdjustmentkD);

        m_straightVelocityControlkP = m_straightVelocityControlkPEntry.getDouble(m_straightVelocityControlkP);
        m_straightVelocityControlkI = m_straightVelocityControlkIEntry.getDouble(m_straightVelocityControlkI);
        m_straightVelocityControlkD = m_straightVelocityControlkDEntry.getDouble(m_straightVelocityControlkD);

        m_targettingTurnKP = m_targettingTurnKPEntry.getDouble(m_targettingTurnKP);
        m_targettingTurnKI = m_targettingTurnKIEntry.getDouble(m_targettingTurnKI);
        m_targettingTurnKD = m_targettingTurnKDEntry.getDouble(m_targettingTurnKD);

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
        return 0.5;
    }

    protected double getPowerCoefficentInitialValue() {
        return .3;
    }

    protected double getRadialTurnCoefficentInitialValue() {
        return 0.05;
    }

    protected double getArcadeLowMaxTurnInitialValue() {
        return 0.25;
    }

    protected double getArcadeHighMaxTurnCoefficentInitalValue() {
        return 1;
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
        return 0.1;
    }

    protected double getStraightVelocityControlkIInitailValue() {
        return 70000000;
    }

    protected double getStraightVelocityControlkDInitailValue() {
        return 0.0;
    }

    protected double getTargettingTurnKPInitialValue() {
        return 0.01;
    }

    protected double getTargettingTurnKIInitialValue() {
        return 0.00;
    }

    protected double getTargettingTurnKDInitialValue() {
        return 0.0001;
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

    public double getPowerCoefficent() {
        if (m_powerCoefficent != null) {
            return m_powerCoefficent;
        }

        return getPowerCoefficentInitialValue();
    }

    public double getRadialTurnCoefficent() {
        if (m_radialTurnCoefficent != null) {
            return m_radialTurnCoefficent;
        }

        return getRadialTurnCoefficentInitialValue();
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

    public double getArcadeHighMaxTurnCoefficent() {
        if (m_arcadeHighMaxTurnCoefficent != null) {
            return m_arcadeHighMaxTurnCoefficent;
        }

        return getArcadeHighMaxTurnCoefficentInitalValue();
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

        return getTalonTankDriveKpInitialValue();
    }

    public double getTalonTankDriveKi() {
        if (m_talonTankDriveKi != null) {
            return m_talonTankDriveKi;
        }

        return getTalonTankDriveKiInitialValue();
    }

    public double getTalonTankDriveKd() {
        if (m_talonTankDriveKd != null) {
            return m_talonTankDriveKd;
        }

        return getTalonTankDriveKdInitialValue();
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

    public double getTargettingTurnKP() {
        if (m_targettingTurnKP != null) {
            return m_targettingTurnKP;
        }

        return getTargettingTurnKPInitialValue();
    }

    public double getTargettingTurnKI() {
        if (m_targettingTurnKI != null) {
            return m_targettingTurnKI;
        }

        return getTargettingTurnKIInitialValue();
    }

    public double getTargettingTurnKD() {
        if (m_targettingTurnKD != null) {
            return m_targettingTurnKD;
        }

        return getTargettingTurnKDInitialValue();
    }

}
