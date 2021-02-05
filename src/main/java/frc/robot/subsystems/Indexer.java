// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Rev Spark Max classes
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// ultra sonic sensor classes
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.IndexerCheckForNewPowerCell;

public class Indexer extends SubsystemBase {
  private double m_TopMotorPower = 0.25;
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

    // test

    m_bottomMotor = new CANSparkMax(3, MotorType.kBrushless);
    m_topMotor = new CANSparkMax(4, MotorType.kBrushless);
    m_bottomMotor.restoreFactoryDefaults();
    m_topMotor.restoreFactoryDefaults();
    m_topMotor.setInverted(true);
    m_bottomMotor.setInverted(true);

    // Try to control how far the balls advance inside indexer
    m_topMotor.setIdleMode(IdleMode.kBrake);
    m_bottomMotor.setIdleMode(IdleMode.kBrake);
    // m_topMotor.setOpenLoopRampRate(1.0);
    // m_bottomMotor.setOpenLoopRampRate(1.0);

    m_encoderBottom = m_bottomMotor.getEncoder();
    m_encoderBottom.setPosition(0);
    m_pidControllerBottom = m_bottomMotor.getPIDController();
    m_pidControllerBottom.setP(m_pid_kP * 1.25); // bottom moter needs a little extra

    m_encoderTop = m_topMotor.getEncoder();
    m_encoderTop.setPosition(0);
    m_pidControllerTop = m_topMotor.getPIDController();
    m_pidControllerTop.setP(m_pid_kP);

    // display variables on SmartDashboard
    SmartDashboard.putBoolean("Indexer/Sensor_0_Intake0", false);
    SmartDashboard.putBoolean("Indexer/Sensor_1_Index1", false);
    SmartDashboard.putBoolean("Indexer/Sensor_2_Index2", false);
    SmartDashboard.putBoolean("Indexer/Sensor_3_Index3", false);
    SmartDashboard.putNumber("Indexer/Sensor_0_Intake Raw", m_Sensor_PC_Intake0.getValue());
    SmartDashboard.putNumber("Indexer/Top Motor Power", m_TopMotorPower);
    SmartDashboard.putNumber("Indexer/Compact Power", m_CompactPower);
    SmartDashboard.putNumber("Indexer/Compact Rotations", m_dCompactRotations);
    SmartDashboard.putNumber("Indexer/Compact Rev Rotations", m_dReverseCompactRotations);

    SmartDashboard.putNumber("Indexer/Compact Encoder Top", m_encoderTop.getPosition());
    SmartDashboard.putNumber("Indexer/Compact Encoder Bottom", m_encoderBottom.getPosition());
    SmartDashboard.putNumber("Indexer/Compact Kp", m_pid_kP);

    CommandBase c = new SequentialCommandGroup(new WaitCommand(0.25), new IndexerCheckForNewPowerCell(this));
    c.setName("Indexer Def Cmd");
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
  }

  // returns true when the index should capture the powercell
  // returns false when no powercell is present
  public boolean SenseIntakePC0() {
    boolean b;

    if (m_Sensor_PC_Intake0.getValue() > 10) {
      b = false;
    } else {
      b = true;
    }
    return b;
  }

  // returns true when the index advance new powercell
  // returns false when indexer/intake is clear
  public boolean SenseIndex1() {
    boolean b;

    if (m_Sensor_PC_Index1.getValue() > 10) {
      b = false;
    } else {
      b = true;
    }
    return b;
  }

  // returns true when the reverse compactor sensor is hit
  // returns false when indexer is clear
  public boolean SenseIndex2() {
    boolean b;

    if (m_Sensor_PC_Index2.getValue() > 10) {
      b = false;
    } else {
      b = true;
    }
    return b;
  }

  // moves the balls to the next sensor
  // returns false if balls are still moving/inbetween
  // returns true if the balls finished moving
  public boolean advance() {
    // set the indexer motors to run
    m_bottomMotor.set(m_TopMotorPower);
    m_topMotor.set(m_TopMotorPower);
    return false;
  }

  // runs until all of the balls are shot out
  // returns false if not all of the balls are out
  // returns true is all of the balls are out
  public boolean shoot() {
    // set the indexer motors to run
    m_bottomMotor.set(0.5);
    m_topMotor.set(0.5);

    // TODO detect whether there are balls remaining
    return false;
  }

  public void stop() {
    // stops the indexer motors
    m_topMotor.set(0);
    m_bottomMotor.set(0);
  }

  // makes the balls go backward out of the intake
  // returns false if the balls did not move back
  // returns true of the balls did move backwards
  public boolean clear() {
    m_bottomMotor.set(-0.25);
    m_topMotor.set(-0.25);
    return false;
  }

  public void resetEncoders() {
    m_encoderTop.setPosition(0);
    m_encoderBottom.setPosition(0);
  }

  // move the balls using the top and bottom motors
  public void compact() {
    // reset encoders and advance distance

    m_pidControllerBottom.setReference(m_dCompactRotations, ControlType.kPosition);
    m_pidControllerTop.setReference(m_dCompactRotations, ControlType.kPosition);
  }

  public void reverseCompact() {
    // m_pidControllerBottom.setReference(m_dReverseCompactRotations,
    // ControlType.kPosition);
    // m_pidControllerTop.setReference(m_dReverseCompactRotations,
    // ControlType.kPosition);

    m_bottomMotor.set(-0.1);
    m_topMotor.set(-0.1);

  }

  // returns true when the indexer is full
  // returns false when indexer has room for more!
  public boolean isFull() {
    boolean b;

    if (m_Sensor_PC_Index3.getValue() > 10) {
      b = false;
    } else {
      b = true;
    }
    return b;
  }
}
