// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Rev Spark Max classes
import com.revrobotics.CANSparkMax;

import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import frc.robot.TheRobot;
import frc.robot.helper.ShooterLookUp;
import frc.robot.helper.ShooterValueSet;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
  private CANSparkMax m_motor1 = null;
  private CANSparkMax m_motor2 = null;
  private CANEncoder m_encoder1 = null;
  private CANPIDController m_pidController = null;
  private DigitalOutput m_LEDrelay = new DigitalOutput(0); // LED ring used for targeting in DIO port 0

  private ShooterLookUp m_lookUpTable = null; // look up table for shooter values

  private double m_RPM_shooter = 0;
  private double m_RPM_target = 5000;
  private double m_RPM_target_range = 100;

  private double m_pid_kIz, m_dpid_kFF;
  private double m_dpid_kI_Modified = .00001;
  private double m_pid_kMaxOutput, m_pid_kMinOutput, m_pid_maxRPM;
  private boolean m_bTuning = false;
  private double m_dTuningRPM = 4000;
  private ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Sub.Shooter");
  private NetworkTableEntry m_testRPM = null;
  private NetworkTableEntry m_actualRPM = null;
  private NetworkTableEntry m_pid_kP = null;
  private NetworkTableEntry m_pid_kI = null;
  private NetworkTableEntry m_pid_kD = null;
  private NetworkTableEntry m_pid_kFF = null;

  private double m_dLowPortRPM = 500;
  private boolean m_bToggleLowPort = false;

  private boolean m_bReadyToShoot = false; // is the shooter ready?
                                           // Here to prevent multiple back ups

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    // TODO fix the CAN id of the motors
    // setup motors
    m_motor1 = new CANSparkMax(2, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(1, MotorType.kBrushless);
    m_motor2.follow(m_motor1, true);
    m_motor1.setSmartCurrentLimit(20, 5500); // limit motor to 20A & 5500 RPM
    m_motor2.setSmartCurrentLimit(20, 5500); // limit motor to 20A & 5500 RPM

    m_encoder1 = m_motor1.getEncoder();
    m_pidController = m_motor1.getPIDController();

    // PID coefficients
    // m_pid_kP = 1e-4;
    // m_pid_kI = 1.75e-7;
    // m_dpid_kI_Modified = setModifiedk.(m_RPM_target, m_pid_kI);
    // m_pid_kD = 0.50;
    m_pid_kIz = 0;
    m_dpid_kFF = 0;
    m_pid_kMaxOutput = 0.8;
    m_pid_kMinOutput = -1;
    m_pid_maxRPM = 5700;

    // set PID coefficients
    // m_pidController.setP(m_pid_kP);
    // m_pidController.setI(m_pid_kI);
    // m_pidController.setD(m_pid_kD);
    m_pidController.setIZone(m_pid_kIz);
    m_pidController.setFF(m_dpid_kFF);
    m_pidController.setOutputRange(m_pid_kMinOutput, m_pid_kMaxOutput);

    m_lookUpTable = new ShooterLookUp();

    // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("Shooter/P Gain", m_pid_kP);
    // SmartDashboard.putNumber("Shooter/I Gain", m_pid_kI);
    SmartDashboard.putNumber("Shooter/I Gain Modified", m_dpid_kI_Modified);
    // SmartDashboard.putNumber("Shooter/D Gain", m_pid_kD);
    SmartDashboard.putNumber("Shooter/I Zone", m_pid_kIz);
    SmartDashboard.putNumber("Shooter/Feed Forward", m_dpid_kFF);
    SmartDashboard.putNumber("Shooter/Max Output", m_pid_kMaxOutput);
    SmartDashboard.putNumber("Shooter/Min Output", m_pid_kMinOutput);
    SmartDashboard.putBoolean("Shooter/Tuning Mode", m_bTuning);
    SmartDashboard.putNumber("Shooter/Tuning RPM", m_dTuningRPM);
    SmartDashboard.putNumber("Shooter/RPM Target Range", m_RPM_target_range);

    this.createShuffleBoardTab();
  }

  public void createShuffleBoardTab() {
    ShuffleboardTab tab = m_shuffleboardTab;
    ShuffleboardLayout shooterCommands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 2)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

    CommandBase c = new frc.robot.commands.Test.Shooter.StartShooter(this);
    c.setName("Start Shooter");
    SmartDashboard.putData(c);
    shooterCommands.add(c);

    c = new frc.robot.commands.Test.Shooter.StopShooter(this);
    c.setName("Stop Shooter");
    SmartDashboard.putData(c);
    shooterCommands.add(c);

    m_testRPM = m_shuffleboardTab.add("Shooter Test RPM", 4000).withWidget(BuiltInWidgets.kNumberSlider).withSize(4, 1)
        .withPosition(2, 0).withProperties(Map.of("min", 0, "max", 10000)).getEntry();

    m_actualRPM = m_shuffleboardTab.add("Shooter Actual RPM", 4000).withWidget(BuiltInWidgets.kGraph)
        .withProperties(Map.of("Visible time", 10)).withSize(4, 4).withPosition(2, 1).getEntry();

    m_pid_kP = m_shuffleboardTab.add("P Gain ", 2.0e-5).withSize(2, 1).withPosition(6, 0).getEntry();

    m_pid_kI = m_shuffleboardTab.add("I Gain", 1.0e-9).withSize(2, 1).withPosition(6, 1).getEntry();

    m_pid_kD = m_shuffleboardTab.add("D Gain", 0).withSize(2, 1).withPosition(6, 2).getEntry();

    m_pid_kFF = m_shuffleboardTab.add("FF Gain", 1.75e-4).withSize(2, 1).withPosition(6, 3).getEntry();

  }

  public double setModifiedkI(double shooterRPM, double unmodifiedPIDkI) {
    double pidModifiedkI = (4000 / shooterRPM) * unmodifiedPIDkI;

    if (pidModifiedkI < 0.0000001) {
      pidModifiedkI = 0.0000001;
    }
    if (pidModifiedkI > 0.0001) {
      pidModifiedkI = 0.0001;
    }

    return pidModifiedkI;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Shooter/P Gain", 0);
    double i = SmartDashboard.getNumber("Shooter/I Gain", 0);
    double d = SmartDashboard.getNumber("Shooter/D Gain", 0);
    double iz = SmartDashboard.getNumber("Shooter/I Zone", 0);
    double ff = SmartDashboard.getNumber("Shooter/Feed Forward", 0);
    double max = SmartDashboard.getNumber("Shooter/Max Output", 0);
    double min = SmartDashboard.getNumber("Shooter/Min Output", 0);
    m_bTuning = SmartDashboard.getBoolean("Shooter/Tuning Mode", m_bTuning);
    m_dTuningRPM = SmartDashboard.getNumber("Shooter/Tuning RPM", m_dTuningRPM);
    m_RPM_target_range = SmartDashboard.getNumber("Shooter/RPM Target Range", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    // if ((p ! = m_pid_kP)) {
    // m_pidController.setP(p);
    // m_pid_kP = p;
    // }
    // if ((i != m_pid_kI)) {
    // m_pid_kI = i;
    // }
    // if ((d != m_pid_kD)) {
    // m_pidController.setD(d);
    // m_pid_kD = d;
    // }
    // if ((iz != m_pid_kIz)) {
    // m_pidController.setIZone(iz);
    // m_pid_kIz = iz;
    // }
    // if ((ff != m_pid_kFF)) {
    // m_pidController.setFF(ff);
    // m_pid_kFF = ff;
    // }
    // if ((max != m_pid_kMaxOutput) || (min != m_pid_kMinOutput)) {
    // m_pidController.setOutputRange(min, max);
    // m_pid_kMinOutput = min;
    // m_pid_kMaxOutput = max;
    // }

    // Get the RPM of the motors
    m_RPM_shooter = Math.abs(m_encoder1.getVelocity());
    m_actualRPM.setDouble(m_RPM_shooter);

    // Output to dashboard
    SmartDashboard.putNumber("Shooter/Current RPM", m_RPM_shooter);
    SmartDashboard.putNumber("Shooter/Target RPM", m_RPM_target);

    SmartDashboard.putNumber("Shooter/I Gain Modified", m_dpid_kI_Modified);
  }

  // shoots the balls
  public void shoot() {
    Robot r = TheRobot.getInstance();

    // get distance to target
    ShooterValueSet m_values = m_lookUpTable.getCurrentValues(true);
    System.out.println(m_values.shooterRPM);
    TheRobot.log("Target RPM: " + TheRobot.toString(m_values.shooterRPM.doubleValue()) + " Target Hood: "
        + TheRobot.toString(m_values.hoodAngle.doubleValue()));

    // tell shooter to come up to target speed based on distance
    if (r.m_shooter.ready(m_values)) {
      // start the indexer
      r.m_indexer.shoot();
    } else {
      // stop the indexer
      // r.m_indexer.stop();
    }
  }

  // runs the shooter backwards in case of a jam
  // returns true if no jam detected
  // returns false if jam detected
  public boolean clear() {
    m_motor1.set(-0.20);
    return false;
  }

  // runs the shooter backwards in case of a jam
  // returns true if no jam detected
  // returns false if jam detected
  public void compact() {
    m_motor1.set(-0.20);
  }

  public void stop() {
    // stops the shooter motors
    m_motor1.set(0);

    // retract the hood
    Robot r = TheRobot.getInstance();
    r.m_hood.retract();
  }

  public void start() {
    // if ((p ! = m_pid_kP)) {
    // m_pidController.setP(p);

    // if ((i != m_pid_kI)) {
    // m_pid_kI = i;

    // if ((d != m_pid_kD)) {
    // m_pidController.setD(d);

    // starts the shooter wheels back up to the reference
    // set the PID Controller to hit the RPM
    double velocity = m_testRPM.getDouble(0);

    double p = m_pid_kP.getDouble(0);
    m_pidController.setP(p);

    double d = m_pid_kD.getDouble(0);
    m_pidController.setD(d);

    double i = m_pid_kI.getDouble(0);
    m_pidController.setI(i);

    double ff = m_pid_kFF.getDouble(0);
    m_pidController.setFF(ff);

    m_pidController.setReference(velocity, ControlType.kVelocity);
    TheRobot.log("Shooter ready RPM_target:" + TheRobot.toString(velocity));
  }

  // returns true if the shooter is up-to-speed for the target distance
  // if distance is zero takes shooter to default speed
  // returns false if the shooter is not at target speed
  public boolean ready(ShooterValueSet m_Values) {
    // set the target RPM
    m_RPM_target = m_Values.shooterRPM;
    if (m_bTuning)
      m_RPM_target = m_dTuningRPM;
    if (m_bToggleLowPort)
      m_RPM_target = m_dLowPortRPM; // If low power port enabled use low RPM

    // set the PID Controller to hit the RPM
    // m_pidController.setReference(m_RPM_target, ControlType.kVelocity);
    TheRobot.log("Shooter ready RPM_target:" + TheRobot.toString(m_RPM_target));

    // See if motor RPM are within range tolerance
    double range = m_RPM_target_range;
    if (Math.abs(m_RPM_target - m_RPM_shooter) < range) {
      Robot r = TheRobot.getInstance();

      // If not targeting and shooter up to speed, then shoot! (return true)

      // TODO fix this block of code in this version of Amperage code
      /*
       * if (r.m_drive.GetTargetingAligned() == false) { return true; }
       * 
       * // Since we are targeting, hold off shooting until we are on target // If on
       * target and we are targeting than shoot! return
       * r.m_drive.GetTargetingAligned();
       */
    }

    return false;
  }

  public void setReadyToShoot(boolean b) {
    m_bReadyToShoot = b;
  }

  public boolean getReadyToShoot() {
    return m_bReadyToShoot;
  }

  public void setLEDRing(Boolean Powered) { // sets the state of the led ring
    m_LEDrelay.set(Powered);
  }

  public void ToggleLowPort() {
    m_bToggleLowPort = !m_bToggleLowPort;
  }

  public boolean isLowPortEnabled() {
    return m_bToggleLowPort;
  }
}
