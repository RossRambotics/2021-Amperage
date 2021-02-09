// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.Year;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.opencv.core.Mat;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveModes.UpdateHandlingCharacteristics;
import frc.robot.helper.DriveHandlingSetup.DefaultHardSurfaceArcadeDrive;
import frc.robot.helper.DriveHandlingSetup.DefaultHardSurfaceHandling;
import frc.robot.helper.DriveHandlingSetup.HandlingBase;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */

  private HandlingBase m_handlingValues;

  // mps = wheel circumference * gearcoefficent * (steps / 100ms) * 1000ms / steps
  // per rotation
  // steps / 100ms = mps * .1 seconds * steps per rotation / wheel cicumference /
  // gear ratio
  private double m_maxVelocity = 20.0; // meters per second
  private double m_velocityCoefficent = 50000.0; // (encoder steps / 100 ms)
  private double m_wheelCircumference = 0.478; // meters - 6 inch diameter
  private double m_gearCoeffiecent = .0933; // 10.71 to 1 falcon rotation to wheel rotation
  private double m_stepsPerRotation = 2048.0; // encoder steps

  private Joystick m_rightLargeJoystick;
  private Joystick m_leftLargeJoystick;
  private Joystick m_smallJoystick;

  private WPI_TalonFX m_rightDriveTalon;
  private WPI_TalonFX m_leftDriveTalon;
  private WPI_TalonFX m_rightDriveTalonFollower;
  private WPI_TalonFX m_leftDriveTalonFollower;
  private TalonFXConfiguration m_leftTalonConfig;
  private TalonFXConfiguration m_rightTalonConfig;

  // private PigeonIMU m_pigeon; // replaced by ADXRS450 gyro
  private ADXRS450_Gyro m_gyro; // the gyro in the SPI port

  private String m_driveProfileSlot = "StraightDrive";

  public Drive(HandlingBase base) {
    m_handlingValues = base;
    System.out.println(base.getMaxDriveOutput());

    // Joysticks
    m_rightLargeJoystick = new Joystick(0);
    m_leftLargeJoystick = new Joystick(1);
    m_smallJoystick = new Joystick(2);

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

    // m_pigeon = new PigeonIMU(31);// new pigeon on 31; doesnt work with falcon 500
    // -- replaced
    m_gyro = new ADXRS450_Gyro();
    m_gyro.reset();

    m_velocityCoefficent = getVelocityCoefficent();
    clearTalonEncoders();
    configureTalons();
    setProfileSlot();
    createShuffleBoardTab();
  }

  public void createShuffleBoardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Sub.Drive");

    ShuffleboardLayout driveModeCommands = tab.getLayout("Set Drive Mode", BuiltInLayouts.kList).withSize(2, 3)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

    CommandBase driveModeSelectCommand = new UpdateHandlingCharacteristics(this, new DefaultHardSurfaceHandling());
    driveModeSelectCommand.setName("Hard Surface Default");
    SmartDashboard.putData(driveModeSelectCommand);
    driveModeCommands.add(driveModeSelectCommand);

    driveModeSelectCommand = new UpdateHandlingCharacteristics(this, new DefaultHardSurfaceArcadeDrive());
    driveModeSelectCommand.setName("Hard Surface Arcade Default");
    SmartDashboard.putData(driveModeSelectCommand);
    driveModeCommands.add(driveModeSelectCommand);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    // m_differentialDrive.tankDrive(leftSpeed, rightSpeed); <-Dumb

    System.out.println("Left: " + leftSpeed + " Right: " + rightSpeed);
    //m_rightDriveTalon.set(ControlMode.Velocity, rightSpeed * m_velocityCoefficent);
    //m_leftDriveTalon.set(ControlMode.Velocity, leftSpeed * m_velocityCoefficent);
  }

  public double getPigeonYaw() {
    return m_gyro.getAngle();
  }

  public void arcadeDrive(double x, double y) {

    if (Math.abs(x) < m_handlingValues.getArcadeLowTurnZone()) { // adjust to make a fine turning zone
      x = m_handlingValues.getArcadeLowTurnCoefficent() * x; // makes the robot turnable at low speeds
    } else {
      x = m_handlingValues.getArcadeHighTurnCoefficent() * x + m_handlingValues.getArcadeLowMaxTurn(); // MAXPOWER
    }

    double leftSpeed = y - x; // acrade drive algorithm
    double rightSpeed = y + x;

    if (leftSpeed > 0) {
      leftSpeed = Math.pow(leftSpeed, 2);
      if (leftSpeed > 1) {
        leftSpeed = 1;
      }
    } else {
      leftSpeed = -Math.pow(leftSpeed, 2);
      if (leftSpeed < -1) {
        leftSpeed = -1;
      }
    }

    if (rightSpeed > 0) {
      rightSpeed = Math.pow(rightSpeed, 2);
      if (rightSpeed > 1) {
        rightSpeed = 1;
      }
    } else {
      rightSpeed = -Math.pow(rightSpeed, 2);
      if (rightSpeed < -1) {
        rightSpeed = -1;
      }
    }

    m_rightDriveTalon.set(ControlMode.Velocity, rightSpeed * m_velocityCoefficent); // sets speeds
    m_leftDriveTalon.set(ControlMode.Velocity, leftSpeed * m_velocityCoefficent);
  }

  private void clearTalonEncoders() // resets the talon encoder positions to 0
  {
    m_leftDriveTalon.getSensorCollection().setIntegratedSensorPosition(0, 100);
    m_leftDriveTalonFollower.getSensorCollection().setIntegratedSensorPosition(0, 100);
    m_rightDriveTalon.getSensorCollection().setIntegratedSensorPosition(0, 100);
    m_rightDriveTalonFollower.getSensorCollection().setIntegratedSensorPosition(0, 100);
  }

  private void configureTalons() {
    m_leftTalonConfig.peakOutputForward = m_handlingValues.getMaxDriveOutput(); // configures the peak outputs
    m_leftTalonConfig.peakOutputReverse = -m_handlingValues.getMaxDriveOutput();
    m_rightTalonConfig.peakOutputForward = m_handlingValues.getMaxDriveOutput();
    m_rightTalonConfig.peakOutputReverse = -m_handlingValues.getMaxDriveOutput();

    // configures the slot0 pids -> switch slots to control different modes
    m_leftTalonConfig.slot0.kP = m_handlingValues.getTalonTankDriveKp();
    m_leftTalonConfig.slot0.kI = m_handlingValues.getTalonTankDriveKi();
    m_leftTalonConfig.slot0.kD = m_handlingValues.getTalonTankDriveKd();

    m_rightTalonConfig.slot0.kP = m_handlingValues.getTalonTankDriveKp();
    m_rightTalonConfig.slot0.kI = m_handlingValues.getTalonTankDriveKi();
    m_rightTalonConfig.slot0.kD = m_handlingValues.getTalonTankDriveKd();

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

  public double getDecelerationDistance(double velocity) // velocity in steps / 100ms
  {
    double mpsVelocity = m_gearCoeffiecent * m_wheelCircumference * velocity * 10 / m_stepsPerRotation;
    double a = 500.0;

    double stoppingDistance = Math.pow(mpsVelocity, 2) * a;

    return stoppingDistance;
  }

  public double getLeftJoystickY() {
    double y = m_leftLargeJoystick.getY();

    if (Math.abs(y) > m_handlingValues.getTankDeadZone()) { // enforces joystick deadzone
      if (Math.abs(y) < m_handlingValues.getTankFineHandlingZone()) { // low velocity for superior handling
        return m_handlingValues.getTankFineHandlingCoefficent() * y;
      } else if (y < 0) { // handles negative
        return Math
            .pow((y + m_handlingValues.getTankFineHandlingZone()) * m_handlingValues.getTankHighPowerCoefficent(), 3)
            - m_handlingValues.getTankFineHandlingMaxVelocity(); // high POWER
      } else {
        return Math
            .pow((y - m_handlingValues.getTankFineHandlingZone()) * m_handlingValues.getTankHighPowerCoefficent(), 3)
            + m_handlingValues.getTankFineHandlingMaxVelocity(); // high POWER
      }
    }

    return 0;
  }

  public double getRightJoystickY() {
    double y = m_rightLargeJoystick.getY();

    if (Math.abs(y) > m_handlingValues.getTankDeadZone()) { // enforces joystick deadzone
      if (Math.abs(y) < m_handlingValues.getTankFineHandlingZone()) { // low velocity for superior handling
        return m_handlingValues.getTankFineHandlingCoefficent() * y;
      } else if (y < 0) { // handles negative
        return Math
            .pow((y + m_handlingValues.getTankFineHandlingZone()) * m_handlingValues.getTankHighPowerCoefficent(), 3)
            - m_handlingValues.getTankFineHandlingMaxVelocity(); // high POWER
      } else {
        return Math
            .pow((y - m_handlingValues.getTankFineHandlingZone()) * m_handlingValues.getTankHighPowerCoefficent(), 3)
            + m_handlingValues.getTankFineHandlingMaxVelocity(); // high POWER
      }
    }

    return 0;
  }

  public double getSmallJoystickX() {
    return m_smallJoystick.getX();
  }

  public double getSmallJoystickY() {
    return m_smallJoystick.getY();
  }

  public boolean getLeftJoystickTrigger() {
    return m_leftLargeJoystick.getRawButton(1);
  }

  public boolean getrightJoystickTrigger() {
    return m_rightLargeJoystick.getRawButton(1);
  }

  public void updateHandlingBase(HandlingBase base) {
    // updates the drive handling characteristics and refreshes talon configs
    m_handlingValues = base;
    configureTalons();
  }

}
