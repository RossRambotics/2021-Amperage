// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.opencv.core.Mat;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */

  private Double m_maxDriveOutput = 0.5;
  private Double m_deadZone = 0.15;
  private Double m_fineHandlingZone = 0.8;
  private Double m_fineHandlingMaxVeloctiy = 0.07;

  // actually get these values
  // ------------------------------------------------------------------
  // mps = wheel circumference * gearcoefficent * (steps / 100ms) * 1000ms / steps
  // per rotation
  // steps / 100ms = mps * .1 seconds * steps per rotation / wheel cicumference /
  // gear ratio
  private Double m_maxVelocity = 20.0; // meters per second
  private Double m_velocityCoefficent = 50000.0; // (encoder steps / 100 ms)
  private Double m_wheelCircumference = 0.478; // meters - 6 inch diameter
  private Double m_gearCoeffiecent = .0933; // 10.71 to 1 falcon rotation to wheel rotation
  private Double m_stepsPerRotation = 2048.0; // encoder steps

  private NetworkTableEntry m_maxDriveOutputEntry = null;

  private Joystick m_rightLargeJoystick;
  private Joystick m_leftLargeJoystick;

  private WPI_TalonFX m_rightDriveTalon;
  private WPI_TalonFX m_leftDriveTalon;
  private WPI_TalonFX m_rightDriveTalonFollower;
  private WPI_TalonFX m_leftDriveTalonFollower;
  private SpeedControllerGroup m_leftDriveGroup;
  private SpeedControllerGroup m_rightDriveGroup;
  private TalonFXConfiguration m_leftTalonConfig;
  private TalonFXConfiguration m_rightTalonConfig;

  private String m_driveProfileSlot = "StraightDrive";

  private DifferentialDrive m_differentialDrive;

  public Drive() {
    // Joysticks
    m_rightLargeJoystick = new Joystick(0);
    m_leftLargeJoystick = new Joystick(1);

    // right side talons
    m_rightDriveTalon = new WPI_TalonFX(21);
    m_rightDriveTalonFollower = new WPI_TalonFX(23);
    m_rightDriveTalonFollower.follow(m_rightDriveTalon);

    // configures the inversion and peak output for the talons
    // m_rightDriveTalon.setInverted(true);
    // m_rightDriveTalonFollower.setInverted(true);

    // left side talons
    m_leftDriveTalon = new WPI_TalonFX(22);
    m_leftDriveTalonFollower = new WPI_TalonFX(24);
    m_leftDriveTalonFollower.follow(m_leftDriveTalon);

    // initialize the configs
    m_leftTalonConfig = new TalonFXConfiguration();
    m_rightTalonConfig = new TalonFXConfiguration();

    // configures the inversion and peak output for the talons
    m_leftDriveTalon.setInverted(true);
    m_leftDriveTalonFollower.setInverted(true);

    // speed controller groups
    // m_leftDriveGroup = new SpeedControllerGroup(m_leftDriveTalon,
    // m_leftDriveTalonFollower);
    // m_rightDriveGroup = new SpeedControllerGroup(m_rightDriveTalon,
    // m_rightDriveTalonFollower);

    // create the differential drive
    // m_differentialDrive = new DifferentialDrive(m_leftDriveGroup,
    // m_rightDriveGroup);

    m_velocityCoefficent = getVelocityCoefficent();

    clearTalonEncoders();

    // setPeakOutputs(m_maxDriveOutput);
    configureTalons();
    setProfileSlot();
    createShuffleBoardTab();
  }

  public void createShuffleBoardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Sub.Indexer");

    ShuffleboardLayout shooterCommands = tab.getLayout("Drive Calibration", BuiltInLayouts.kList).withSize(2, 2)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

    m_maxDriveOutputEntry = shooterCommands.add("MAX POWER!", 0.5).withWidget(BuiltInWidgets.kNumberSlider)
        .withSize(4, 1).withProperties(Map.of("min", 0, "max", 1)).getEntry();
  }

  @Override
  public void periodic() {

    if (m_maxDriveOutput != m_maxDriveOutputEntry.getDouble(0.5)) {
      m_maxDriveOutput = m_maxDriveOutputEntry.getDouble(0.5);
      setPeakOutputs(m_maxDriveOutput);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void tankDrive(Double leftSpeed, Double rightSpeed) {
    // m_differentialDrive.tankDrive(leftSpeed, rightSpeed); <-Dumb

    System.out.println(rightSpeed);
    System.out.println(rightSpeed * m_velocityCoefficent);
    m_rightDriveTalon.set(ControlMode.Velocity, rightSpeed * m_velocityCoefficent);
    m_leftDriveTalon.set(ControlMode.Velocity, leftSpeed * m_velocityCoefficent);
  }

  private void setPeakOutputs(Double peakOutput) // sets the max peak outputs on a motor to prevent a brownout
  {
    m_leftDriveTalon.configPeakOutputForward(peakOutput);
    m_leftDriveTalon.configPeakOutputReverse(-peakOutput);
    m_leftDriveTalonFollower.configPeakOutputForward(peakOutput); // you must set this on followers too...
    m_leftDriveTalonFollower.configPeakOutputReverse(-peakOutput); // you must set this on followers too...

    m_rightDriveTalon.configPeakOutputForward(peakOutput);
    m_rightDriveTalon.configPeakOutputReverse(-peakOutput);
    m_rightDriveTalonFollower.configPeakOutputForward(peakOutput); // you must set this on followers too...
    m_rightDriveTalonFollower.configPeakOutputReverse(-peakOutput); // you must set this on followers too...
  }

  private void clearTalonEncoders() // resets the talon encoder positions to 0
  {
    m_leftDriveTalon.getSensorCollection().setIntegratedSensorPosition(0, 100);
    m_leftDriveTalonFollower.getSensorCollection().setIntegratedSensorPosition(0, 100);
    m_rightDriveTalon.getSensorCollection().setIntegratedSensorPosition(0, 100);
    m_rightDriveTalonFollower.getSensorCollection().setIntegratedSensorPosition(0, 100);
  }

  private void configureTalons() {
    m_leftTalonConfig.peakOutputForward = 0.5; // configures the peak outputs
    m_leftTalonConfig.peakOutputReverse = -0.5;
    m_rightTalonConfig.peakOutputForward = 0.5;
    m_rightTalonConfig.peakOutputReverse = -0.5;

    m_leftTalonConfig.slot0.kP = 0.1; // configures the slot0 pids -> switch slots to control different modes
    m_leftTalonConfig.slot0.kI = 0;
    m_leftTalonConfig.slot0.kD = 0;

    m_rightTalonConfig.slot0.kP = 0.1;
    m_rightTalonConfig.slot0.kI = 0;
    m_rightTalonConfig.slot0.kD = 0;

    m_leftDriveTalon.configAllSettings(m_leftTalonConfig);
    m_rightDriveTalon.configAllSettings(m_rightTalonConfig);
    m_leftDriveTalonFollower.configAllSettings(m_leftTalonConfig);
    m_rightDriveTalonFollower.configAllSettings(m_rightTalonConfig);
  }

  private void setProfileSlot() {
    switch (m_driveProfileSlot) { // selects the drive mode PID
      case "StraightDrive": // the mode for straight drives
        System.out.println("Drive: TankDrive profile selected");
        m_leftDriveTalon.selectProfileSlot(0, 0);
        m_rightDriveTalon.selectProfileSlot(0, 0);
        break;

      default:
        System.out.println("Drive: TankDrive profile selected by default");
        m_leftDriveTalon.selectProfileSlot(0, 0);
        m_rightDriveTalon.selectProfileSlot(0, 0);
        break;
    }
  }

  public double getLeftTalonEncoderPosition() {
    return m_leftDriveTalon.getSensorCollection().getIntegratedSensorPosition(); // average of master and follower?
  }

  public double getRightTalonEncoderPostion() {
    return m_rightDriveTalon.getSensorCollection().getIntegratedSensorPosition(); // average of master and follower?
  }

  public void setTargetLeftTalonTargetPosition(double targetSteps) {
    m_leftDriveTalon.set(ControlMode.Position, targetSteps);
  }

  public void setTargetRightTalonTargetPosition(double targetSteps) {
    m_leftDriveTalon.set(ControlMode.Position, targetSteps);
  }

  public double getVelocityCoefficent() // gets the scaling factor to translate the joystick input to the velcoity input
  {
    return (m_maxVelocity * 0.1 * m_stepsPerRotation / m_gearCoeffiecent / m_wheelCircumference);
    // mps = wheel circumference * gearcoefficent * (steps / 100 ms) * 1000ms /
    // steps per rotation
    // steps / 100ms = mps * .1 seconds * steps per rotation / wheel cicumference /
    // gear ratio
  }

  public double getDistancePerStep() // meters / step
  {
    return m_wheelCircumference * m_gearCoeffiecent / m_stepsPerRotation;
  }

  public double getDecelerationDistance(Double velocity) // velocity in steps / 100ms
  {
    double mpsVelocity = m_gearCoeffiecent * m_wheelCircumference * velocity * 10 / m_stepsPerRotation;
    double a = 500.0;

    double stoppingDistance = Math.pow(mpsVelocity, 2) * a;

    return stoppingDistance;
  }

  public double getLeftJoystickY() {
    double y = m_leftLargeJoystick.getY();

    if (Math.abs(y) > m_deadZone) {
      if (Math.abs(y) < m_fineHandlingZone) {
        double fineHandlingCoefficent = m_fineHandlingMaxVeloctiy / (m_fineHandlingZone - m_deadZone);
        return fineHandlingCoefficent * y;
      }

      double highPowerCoefficent = (1 - .1) / (1 - m_fineHandlingZone);
      return Math.pow(y * highPowerCoefficent, 3) + 0.1;
    }

    return 0;
  }

  public double getRightJoystickY() {
    double y = m_rightLargeJoystick.getY();

    if (Math.abs(y) > m_deadZone) {
      if (Math.abs(y) < m_fineHandlingZone) {
        double fineHandlingCoefficent = m_fineHandlingMaxVeloctiy / (m_fineHandlingZone - m_deadZone);
        return fineHandlingCoefficent * y;
      }

      double highPowerCoefficent = (1 - .1) / (1 - m_fineHandlingZone);
      return Math.pow(y * highPowerCoefficent, 3) + 0.1;
    }

    return 0;
  }

  public boolean getLeftJoystickTrigger() {
    return m_leftLargeJoystick.getRawButton(1);
  }

  public boolean getrightJoystickTrigger() {
    return m_rightLargeJoystick.getRawButton(1);
  }

}
