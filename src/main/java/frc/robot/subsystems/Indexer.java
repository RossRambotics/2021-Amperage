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

  private AnalogInput m_indexerlight = new AnalogInput(1);
  private AnalogInput m_backtoplight = new AnalogInput(0);
  private AnalogInput m_fronttoplight = new AnalogInput(4);

  // public CANSparkMax m_intake = new CANSparkMax(11, MotorType.kBrushless);
  public CANSparkMax m_topMotor = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax m_btmMotor = new CANSparkMax(3, MotorType.kBrushless);

  // public CANSparkMax m_shooter = new CANSparkMax(1, MotorType.kBrushless);
  // public CANSparkMax m_shooter2 = new CANSparkMax(2, MotorType.kBrushless);
  /** Creates a new Indexer. */
  public Indexer() {

    // setting default command to check for powercell
    CommandBase c = new SequentialCommandGroup(new WaitCommand(0.1), new CheckPowercell(this));
    c.setName("Indexer's DefCMD");
    this.setDefaultCommand(c);

    m_btmMotor.restoreFactoryDefaults();
    m_topMotor.restoreFactoryDefaults();
    m_topMotor.setInverted(true);
    m_btmMotor.setInverted(true);

    this.createShuffleBoardTab();
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

  public boolean checkIndexerSensor() {
    // TheRobot.log("FirstSensor: " + m_indexerlight.getValue());

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

  public void reverse() {
    m_topMotor.set(-0.2);
    m_btmMotor.set(-0.2);
  }

  public void shoot() {
    m_topMotor.set(0.50);
    m_btmMotor.set(0.50);
  }

  public void advance() {
    m_topMotor.set(0.27);
    m_btmMotor.set(0.27);
  }

  public void stop() {
    m_topMotor.set(0);
    m_btmMotor.set(0);
  }

}