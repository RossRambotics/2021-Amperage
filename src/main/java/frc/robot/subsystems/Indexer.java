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
import frc.robot.commands.Indexer.IndexerDefaultCommand;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Indexer extends SubsystemBase {

  // the indexer front and rear could easily be reversed
  private AnalogInput m_rearIndexerSensor = new AnalogInput(1);
  private AnalogInput m_frontIndexerSensor = new AnalogInput(2);
  private AnalogInput m_backtoplight = new AnalogInput(0);
  private AnalogInput m_fronttoplight = new AnalogInput(3);

  public CANSparkMax m_topMotor = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax m_btmMotor = new CANSparkMax(3, MotorType.kBrushless);

  private Timer m_currentTimer = new Timer();
  private boolean m_compactAvaliable = true; // if the indexer is allowed to compact
  private boolean m_enableIndexerAdvance = true;
  private double m_compactEndTime = 0; // the time the compact sequence can no longer run
  private double m_compactDuration = 1; // the time the compactor is allowed to run for in seconds

  /** Creates a new Indexer. */
  public Indexer() {

    // setting default command to check for powercell
    // consider adding a wait?
    this.setDefaultCommand(new IndexerDefaultCommand(this, TheRobot.getInstance().m_intake));

    m_btmMotor.restoreFactoryDefaults();
    m_topMotor.restoreFactoryDefaults();
    m_topMotor.setInverted(true);
    m_btmMotor.setInverted(true);

    this.createShuffleBoardTab();

    m_currentTimer.start();
  }

  public void createShuffleBoardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Sub.Indexer");

    ShuffleboardLayout shooterCommands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 2)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

    CommandBase c = new frc.robot.commands.Indexer.StartIndexer(this);
    c.setName("Start Indexer");
    SmartDashboard.putData(c);
    shooterCommands.add(c);

    c = new frc.robot.commands.Indexer.StopIndexer(this);
    c.setName("Stop Indexer");
    SmartDashboard.putData(c);
    shooterCommands.add(c);

    // TODO make the timeout value a variable on the tab
    c = new frc.robot.commands.Indexer.RunIndexer(this).withTimeout(0.5);
    c.setName("Run Indexer");
    SmartDashboard.putData(c);
    shooterCommands.add(c);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(m_indexerlight.getValue() < 10 ){
    // System.out.println("Object detected");
    // System.out.println(m_indexerlight.getValue());
    // }
    // m_indexerMotor.set(0.25);
    // m_intake.set(0.25);
    // m_topMotor.set(0.25);
    // m_btmMotor.set(0.25);
    // m_shooter.set(0.25);
    // m_shooter2.set(1);
  }

  public boolean getIndexerRearSensor() {
    // the one near the intake
    // TheRobot.log("FirstSensor: " + m_indexerlight.getValue());

    if (m_rearIndexerSensor.getValue() < 10) {
      SmartDashboard.putBoolean("Indexer/IndexerLight", true);
      return true;
    } else {
      SmartDashboard.putBoolean("Indexer/IndexerLight", false);
      return false;
    }
  }

  public boolean getIndexerFrontSensor() {
    // the one near the shooter
    // TheRobot.log("FirstSensor: " + m_indexerlight.getValue());

    if (m_frontIndexerSensor.getValue() < 10) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getIntakeIndexerSensor() {
    if (m_backtoplight.getValue() < 10) {
      SmartDashboard.putBoolean("Indexer/BackTopLight", true);
      return true;
    } else {
      SmartDashboard.putBoolean("Indexer/BackTopLight", false);
      return false;
    }
  }

  public boolean getShooterIndexerSensor() {
    if (m_fronttoplight.getValue() < 10) {
      SmartDashboard.putBoolean("Indexer/FrontTopLight", true);
      return true;
    } else {
      SmartDashboard.putBoolean("Indexer/FrontTopLight", false);
      return false;
    }
  }

  public void reverse() {
    m_topMotor.set(-0.2);
    m_btmMotor.set(-0.2);
  }

  public void reverseSlow() {
    m_topMotor.set(-0.1);
    m_btmMotor.set(-0.1);
  }

  public void shoot() {
    m_topMotor.set(0.25);
    m_btmMotor.set(0.25);
  }

  public void advance() {
    m_topMotor.set(0.18);
    m_btmMotor.set(0.18);
  }

  public void stop() {
    m_topMotor.set(0);
    m_btmMotor.set(0);
  }

  public void compact() {
    if (m_compactAvaliable) { // if the compact is avaliable to start
      m_compactAvaliable = false; // disable after start

      m_compactEndTime = m_currentTimer.get() + m_compactDuration;
    }

    if (m_currentTimer.get() < m_compactEndTime) { // if the compactor should run
      reverseSlow();
    } else { // if the time is up on the compact sequence stop
      stop();
    }
  }

  public void enableCompact() {
    m_compactAvaliable = true;
  }

  public void setIndexerAdvanceEnable(boolean value) {
    m_enableIndexerAdvance = value;
  }

  public boolean getIndexerAdvanceEnabled() {
    return m_enableIndexerAdvance;
  }

}