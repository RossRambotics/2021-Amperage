// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.TheRobot;

import java.util.Map;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(31, 0, 1);// creates the solenoid on CAN id 15
  private CANSparkMax m_rollerMotor = null;
  private CANSparkMax m_feederMotor = null;
  private CANEncoder intakeEncoder = null;
  private CANPIDController intakePIDController = null;
  private DigitalOutput m_LEDrelay = new DigitalOutput(1); // LED ring used for targeting in DIO port 1

  private Double pid_kP;
  private Double pid_kI;
  private Double pid_kD;
  private Double pid_kIzone;
  private Double pid_kFF;
  private Double pid_kMAX;
  private Double pid_kMIN;

  private Double captureSpeed = 0.5; // the capture speed for the intake in RPM
  private boolean m_bExtended = false;

  public Intake() {
    m_rollerMotor = new CANSparkMax(11, MotorType.kBrushless);
    m_feederMotor = new CANSparkMax(12, MotorType.kBrushless);

    m_rollerMotor.restoreFactoryDefaults();
    intakeEncoder = new CANEncoder(m_rollerMotor);

    intakeSolenoid.set(Value.kForward); // retract the intake

    intakeEncoder.setPosition(0);
    SmartDashboard.putNumber("Intake/CaptureSpeed", captureSpeed);
    SmartDashboard.putBoolean("Intake/Extended?", m_bExtended);
    /*
     * intakePIDController = m_rollerMotor.getPIDController();
     * 
     * pid_kP = 0.0001; pid_kI = 0.0; pid_kD = 0.0; pid_kIzone = 0.0; pid_kFF = 0.0;
     * pid_kMAX = 1.0; pid_kMIN = -1.0;
     * 
     * captureSpeed = 600.0;
     * 
     * intakePIDController.setP(pid_kP); intakePIDController.setI(pid_kI);
     * intakePIDController.setD(pid_kD); intakePIDController.setIZone(pid_kIzone);
     * intakePIDController.setFF(pid_kFF);
     * intakePIDController.setOutputRange(pid_kMIN, pid_kMAX);
     * 
     * SmartDashboard.putNumber("Intake/CaptureSpeed", captureSpeed);
     * SmartDashboard.putNumber("Intake/pid_kP", pid_kP);
     * SmartDashboard.putNumber("Intake/pid_kI", pid_kI);
     * SmartDashboard.putNumber("Intake/pid_kD", pid_kD);
     * SmartDashboard.putNumber("Intake/pid_kIzone", pid_kIzone);
     * SmartDashboard.putNumber("Intake/pid_kFF", pid_kFF);
     * SmartDashboard.putNumber("Intake/pid_kMAX", pid_kMAX);
     * SmartDashboard.putNumber("Intake/pid_kMIN", pid_kMIN);
     * SmartDashboard.putNumber("Intake/rollerMotor_Power", m_rollerMotor.get());
     */

  }

  public void createShuffleBoardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Sub.Intake");
    ShuffleboardLayout intakeCommands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 2)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

    CommandBase c = new frc.robot.commands.Test.Intake.StartFeederWheels(this);
    c.setName("Start Feeder Wheels");
    SmartDashboard.putData(c);
    intakeCommands.add(c);

    c = new frc.robot.commands.Test.Intake.StopFeederWheels(this);
    c.setName("Stop Feeder Wheels");
    SmartDashboard.putData(c);
    intakeCommands.add(c);

    c = new frc.robot.commands.Test.Intake.StartIntakeRoller(this);
    c.setName("Start Intake Roller");
    SmartDashboard.putData(c);
    intakeCommands.add(c);

    c = new frc.robot.commands.Test.Intake.StopIntakeRoller(this);
    c.setName("Stop Intake Roller");
    SmartDashboard.putData(c);
    intakeCommands.add(c);

    // tab.add("Start Feeder Wheels", c);

    // tab.add("RPM Target",
    // 1).withWidget(BuiltInWidgets.kNumberSlider).withPosition(3, 3)
    // .withProperties(Map.of("min", 0, "max", 10000)); // specify
    // // widget
    // // properties
    // // here

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/rollerMotor_Power", m_rollerMotor.get());
    SmartDashboard.putNumber("Intake/rollerMotor Current", m_rollerMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Intake/Extended?", m_bExtended);
    captureSpeed = SmartDashboard.getNumber("Intake/CaptureSpeed", 0);

    if (true)
      return;
    // updates SmartDashbaord linked variables as needed
    if (captureSpeed != SmartDashboard.getNumber("Intake/CaptureSpeed", captureSpeed)) {
      captureSpeed = SmartDashboard.getNumber("Intake/CaptureSpeed", captureSpeed);
    }
    if (pid_kP != SmartDashboard.getNumber("Intake/pid_kP", pid_kP)) {
      pid_kP = SmartDashboard.getNumber("Intake/pid_kP", pid_kP);
      intakePIDController.setP(pid_kP);
    }
    if (pid_kI != SmartDashboard.getNumber("Intake/pid_kI", pid_kI)) {
      pid_kI = SmartDashboard.getNumber("Intake/pid_kI", pid_kI);
      intakePIDController.setI(pid_kI);
    }
    if (pid_kD != SmartDashboard.getNumber("Intake/pid_kD", pid_kD)) {
      pid_kD = SmartDashboard.getNumber("Intake/pid_kD", pid_kD);
      intakePIDController.setD(pid_kD);
    }
    if (pid_kIzone != SmartDashboard.getNumber("Intake/pid_kIzone", pid_kIzone)) {
      pid_kIzone = SmartDashboard.getNumber("Intake/pid_kIzone", pid_kIzone);
      intakePIDController.setIZone(pid_kIzone);
    }
    if (pid_kFF != SmartDashboard.getNumber("Intake/pid_kFF", pid_kFF)) {
      pid_kFF = SmartDashboard.getNumber("Intake/pid_kFF", pid_kFF);
      intakePIDController.setFF(pid_kFF);
    }
    if (pid_kMAX != SmartDashboard.getNumber("Intake/pid_kMAX", pid_kMAX)
        | pid_kMIN != SmartDashboard.getNumber("Intake/pid_kMIN", pid_kMIN)) {
      pid_kMAX = SmartDashboard.getNumber("Intake/pid_kMIN", pid_kMAX);
      pid_kMIN = SmartDashboard.getNumber("Intake/pid_kMIN", pid_kMIN);
      intakePIDController.setOutputRange(pid_kMIN, pid_kMAX);
    }

    // This method will be called once per scheduler run
  }

  // retracts the intake
  public void retract() {
    intakeSolenoid.set(Value.kForward);
    m_bExtended = false;
  }

  // extends the intake
  public void extend() {
    intakeSolenoid.set(Value.kReverse);
    m_bExtended = true;
  }

  // spins the intake to capture a ball
  public void capture() {

    // intakePIDController.setReference(captureSpeed, ControlType.kVelocity);
    TheRobot.log("Starting Intake Motor");
    m_rollerMotor.set(captureSpeed);

  }

  // Stops the capture process
  public void stopCapture() {
    // intakePIDController.setReference(0, ControlType.kVelocity);
    m_rollerMotor.set(0.0);
  }

  // reverse spins the intake in case of jam
  // return true if cleared
  // return false if jam is still detected
  public void clear() {

    TheRobot.log("Clearing Intake Motor");
    m_rollerMotor.set(-captureSpeed);
  }

  public boolean isExtended() {

    return m_bExtended;
  }

  public void setLEDRing(Boolean Powered) { // sets the state of the led ring
    m_LEDrelay.set(Powered);
  }

  public void startFeederWheels() {
    m_feederMotor.set(0.5);
  }

  public void stopFeederWheels() {
    m_feederMotor.set(0.0);
  }

  public void startIntakeRoller() {
    m_rollerMotor.set(0.5);
  }

  public void stopIntakeRoller() {
    m_rollerMotor.set(0.0);
  }

}
