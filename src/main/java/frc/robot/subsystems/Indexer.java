// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.TheRobot;
import frc.robot.commands.Indexer.CheckPowercell;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.IndexerCheckForNewPowerCell;

public class Indexer extends SubsystemBase {
  private double m_TopMotorPower = 1;
  private double m_CompactPower = 0.10;
  private double m_dCompactRotations = 1.0;
  private double m_dReverseCompactRotations = 0.5;
  private CANSparkMax m_bottomMotor = null;
  private CANSparkMax m_topMotor = null;
  private CANEncoder m_encoderBottom = null;
  private CANPIDController m_pidControllerBottom = null;
  private CANEncoder m_encoderTop = null;
  private CANPIDController m_pidControllerTop = null;
  private double m_pid_kP = 0.2;
  private double m_pid_kI = 0.001;
  private boolean m_bReadyToShoot = false;
  private DoubleSolenoid m_stopperSolenoid;

  // setup ultra sonic sensor
  public AnalogInput m_Sensor_PC_Intake0 = new AnalogInput(0); // intake has presented powercell to indexer
  public AnalogInput m_Sensor_PC_Index1 = new AnalogInput(1); // intake has presented powercell to indexer
  public AnalogInput m_Sensor_PC_Index2 = new AnalogInput(2); // reverse compact sensor
  public AnalogInput m_Sensor_PC_Index3 = new AnalogInput(3); // indexer full sensor

  // public AnalogInput m_Sensor_PC_Capture = new AnalogInput(1); // ball as been
  // captured in indexer

  /**
   * Creates a new Indexer.
   */
  public Indexer() {


import java.util.Map;

    m_stopperSolenoid = new DoubleSolenoid(31, 2, 4);
    m_stopperSolenoid.set(Value.kForward);
    // Try to control how far the balls advance inside indexer
    m_topMotor.setIdleMode(IdleMode.kBrake);
    m_bottomMotor.setIdleMode(IdleMode.kBrake);
    // m_topMotor.setOpenLoopRampRate(1.0);
    // m_bottomMotor.setOpenLoopRampRate(1.0);

public class Indexer extends SubsystemBase {

  private AnalogInput m_indexerlight = new AnalogInput(1);
  private AnalogInput m_backtoplight = new AnalogInput(0);
  private AnalogInput m_fronttoplight = new AnalogInput(4);
  public CANSparkMax m_indexerMotor = new CANSparkMax(12, MotorType.kBrushless);

  // public CANSparkMax m_intake = new CANSparkMax(11, MotorType.kBrushless);
  public CANSparkMax m_topMotor = new CANSparkMax(3, MotorType.kBrushless);
  public CANSparkMax m_btmMotor = new CANSparkMax(4, MotorType.kBrushless);

  // public CANSparkMax m_shooter = new CANSparkMax(1, MotorType.kBrushless);
  // public CANSparkMax m_shooter2 = new CANSparkMax(2, MotorType.kBrushless);
  /** Creates a new Indexer. */
  public Indexer() {

    // setting default command to check for powercell
    CommandBase c = new SequentialCommandGroup(new WaitCommand(0.1), new CheckPowercell(this));
    c.setName("Indexer's DefCMD");
    this.setDefaultCommand(c);

    this.createShuffleBoardTab();
  }

  public void createShuffleBoardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Sub.Indexer");

    ShuffleboardLayout shooterCommands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 2)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

    CommandBase c = new frc.robot.commands.Test.Indexer.StartIndexer(this);
    c.setName("Start Indexer");
    SmartDashboard.putData(c);
    shooterCommands.add(c);

    c = new frc.robot.commands.Test.Indexer.StopIndexer(this);
    c.setName("Stop Indexer");
    SmartDashboard.putData(c);
    shooterCommands.add(c);

    // TODO make the timeout value a variable on the tab
    c = new frc.robot.commands.Test.Indexer.RunIndexer(this).withTimeout(0.5);
    c.setName("Run Indexer");
    SmartDashboard.putData(c);
    shooterCommands.add(c);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // display variables on SmartDashboard
    boolean b = (m_Sensor_PC_Intake0.getValue() > 10) ? false : true;
    SmartDashboard.putBoolean("Indexer/Sensor_0_Intake0", b);
    SmartDashboard.putNumber("Indexer/Sensor_0_Intake Raw", m_Sensor_PC_Intake0.getValue());

    b = (m_Sensor_PC_Index1.getValue() > 10) ? false : true;
    SmartDashboard.putBoolean("Indexer/Sensor_1_Index1", b);

    b = (m_Sensor_PC_Index2.getValue() > 10) ? false : true;
    SmartDashboard.putBoolean("Indexer/Sensor_2_Index2", b);

    b = (m_Sensor_PC_Index3.getValue() > 10) ? false : true;
    SmartDashboard.putBoolean("Indexer/Sensor_3_Index3", b);

    SmartDashboard.putNumber("Indexer/Compact Encoder Top", m_encoderTop.getPosition());
    SmartDashboard.putNumber("Indexer/Compact Encoder Bottom", m_encoderBottom.getPosition());
    double Kp = SmartDashboard.getNumber("Indexer/Compact Kp", 0);
    if (Kp != m_pid_kP) {
      m_pid_kP = Kp;
      m_pidControllerTop.setP(m_pid_kP);
      m_pidControllerBottom.setP(m_pid_kP);
    }

    m_TopMotorPower = SmartDashboard.getNumber("Indexer/Top Motor Power", 0);
    m_CompactPower = SmartDashboard.getNumber("Indexer/Compact Power", 0);
    m_dCompactRotations = SmartDashboard.getNumber("Indexer/Compact Rotations", 0);
    m_dReverseCompactRotations = SmartDashboard.getNumber("Indexer/Rev Compact Rotations", 0);

    if (m_bReadyToShoot) { // control indexer compaction cylinder
      m_stopperSolenoid.set(Value.kReverse); // retracts compaction soleoid

    } else {
      m_stopperSolenoid.set(Value.kForward); // extends retraction solenoid

    }
  }

  public boolean checkIndexerSensor() {
    TheRobot.log("FirstSensor: " + m_indexerlight.getValue());

    if (m_indexerlight.getValue() < 10) {
      SmartDashboard.putBoolean("Indexer/IndexerLight", true);
      return true;
    } else {
      SmartDashboard.putBoolean("Indexer/IndexerLight", false);
      return false;
    }
  }

  public boolean checkBackTopLight() {
    if (m_backtoplight.getValue() < 10) {
      SmartDashboard.putBoolean("Indexer/BackTopLight", true);
      return true;
    } else {
      SmartDashboard.putBoolean("Indexer/BackTopLight", false);
      return false;
    }
  }

  public boolean checkFrontTopLight() {
    if (m_fronttoplight.getValue() < 10) {
      SmartDashboard.putBoolean("Indexer/FrontTopLight", true);
      return true;
    } else {
      SmartDashboard.putBoolean("Indexer/FrontTopLight", false);
      return false;
    }
  }

  // moves the balls to the next sensor
  // returns false if balls are still moving/inbetween
  // returns true if the balls finished moving
  public boolean advance() {
    // set the indexer motors to run
    m_bottomMotor.set(m_TopMotorPower);
    m_topMotor.set(-m_TopMotorPower);
    return false;
  }
  
  public void shoot() {

  }


  public void stop() {
    m_topMotor.set(0);
    m_btmMotor.set(0);
  }

}
