// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.Year;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.opencv.core.Mat;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
import frc.robot.helper.Targetting.ShooterLookUp;
import frc.robot.helper.DriveHandlingSetup.*;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */

  public HandlingBase m_handlingValues;
  public ShooterLookUp m_shooterLookUpTable;

  // mps = wheel circumference * gearcoefficent * (steps / 100ms) * 1000ms / steps
  // per rotation
  // steps / 100ms = mps * .1 seconds * steps per rotation / wheel cicumference /
  // gear ratio
  private double m_velocityCoefficent = 50000.0; // (encoder steps / 100 ms)
  private double m_wheelCircumference = 0.478; // meters - 6 inch diameter
  private double m_gearCoeffiecent = .0933; // 10.71 to 1 falcon rotation to wheel rotation
  private double m_stepsPerRotation = 2048.0; // encoder steps
  private double m_trackwidth = 0.556; // the robot trackwidth7
  private double m_degreesFrameRotationPerStep; // the degrees of frameRoation per step

  private Joystick m_rightLargeJoystick;
  private Joystick m_leftLargeJoystick;
  private Joystick m_smallJoystick;

  private WPI_TalonFX m_rightDriveTalon;
  private WPI_TalonFX m_leftDriveTalon;
  private WPI_TalonFX m_rightDriveTalonFollower;
  private WPI_TalonFX m_leftDriveTalonFollower;
  private TalonFXConfiguration m_leftTalonConfig;
  private TalonFXConfiguration m_rightTalonConfig;
  private TalonFXSensorCollection m_rightTalonSensors;
  private TalonFXSensorCollection m_leftTalonSensors;

  // private PigeonIMU m_pigeon; // replaced by ADXRS450 gyro
  private ADXRS450_Gyro m_gyro; // the gyro in the SPI port
  private PowerDistributionPanel m_PDP; // the PDP

  private String m_driveProfileSlot = "StraightDrive";

  public Drive(HandlingBase base) {
    m_handlingValues = base;
    System.out.println("MaxDrive: " + base.getMaxDriveOutput());

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

    // gets the sensors
    m_rightTalonSensors = m_rightDriveTalon.getSensorCollection();
    m_leftTalonSensors = m_leftDriveTalon.getSensorCollection();

    // m_pigeon = new PigeonIMU(31);// new pigeon on 31; doesnt work with falcon 500
    // -- replaced
    m_gyro = new ADXRS450_Gyro();
    m_gyro.reset();

    m_PDP = new PowerDistributionPanel(30);

    m_shooterLookUpTable = new ShooterLookUp();

    m_degreesFrameRotationPerStep = getDegreesFrameRotationPerStep();

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

    driveModeSelectCommand = new UpdateHandlingCharacteristics(this, new TyTankDrive());
    driveModeSelectCommand.setName("Ty Tank Drive");
    SmartDashboard.putData(driveModeSelectCommand);
    driveModeCommands.add(driveModeSelectCommand);

    driveModeSelectCommand = new UpdateHandlingCharacteristics(this, new DerekTankDrive());
    driveModeSelectCommand.setName("Derek Tank Drive");
    SmartDashboard.putData(driveModeSelectCommand);
    driveModeCommands.add(driveModeSelectCommand);

    driveModeSelectCommand = new UpdateHandlingCharacteristics(this, new ChesterTankDrive());
    driveModeSelectCommand.setName("Chester Tank Drive");
    SmartDashboard.putData(driveModeSelectCommand);
    driveModeCommands.add(driveModeSelectCommand);

    driveModeSelectCommand = new UpdateHandlingCharacteristics(this, new AndrewArcadeDrive());
    driveModeSelectCommand.setName("Andrew Arcade Drive");
    SmartDashboard.putData(driveModeSelectCommand);
    driveModeCommands.add(driveModeSelectCommand);

    driveModeSelectCommand = new UpdateHandlingCharacteristics(this, new WillArcadeDrive());
    driveModeSelectCommand.setName("Will Arcade Drive");
    SmartDashboard.putData(driveModeSelectCommand);
    driveModeCommands.add(driveModeSelectCommand);

    driveModeSelectCommand = new UpdateHandlingCharacteristics(this, new WillTankDrive());
    driveModeSelectCommand.setName("Will Tank Drive");
    SmartDashboard.putData(driveModeSelectCommand);
    driveModeCommands.add(driveModeSelectCommand);

    driveModeSelectCommand = new UpdateHandlingCharacteristics(this, new RyanTankDrive());
    driveModeSelectCommand.setName("Ryan Arcade Drive");
    SmartDashboard.putData(driveModeSelectCommand);
    driveModeCommands.add(driveModeSelectCommand);

  }

  @Override
  public void periodic() {
    m_handlingValues.refreshNetworkTablesValues();
    // configureTalons();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    // m_differentialDrive.tankDrive(leftSpeed, rightSpeed); <-Dumb

    m_velocityCoefficent = getVelocityCoefficent();

    m_rightDriveTalon.set(ControlMode.Velocity, rightSpeed * m_velocityCoefficent);
    m_leftDriveTalon.set(ControlMode.Velocity, leftSpeed * m_velocityCoefficent);
  }

  public void tankDriveRaw(double leftSpeed, double rightSpeed) { // tank drive but without velocity adjustment
    m_rightDriveTalon.set(ControlMode.PercentOutput, rightSpeed);
    m_leftDriveTalon.set(ControlMode.PercentOutput, leftSpeed);
  }

  public boolean getSmallJoystickLeftShoulder() {
    return m_smallJoystick.getRawButton(5);
  }

  public boolean getSmallJoystickRightShoulder() {
    return m_smallJoystick.getRawButton(6);
  }

  public double getGyroYaw() {
    return m_gyro.getAngle();
  }

  public void arcadeDrive(double x, double y, String brakeSide) {
    // brakeSide = -1 stop left, 1 stop right, 0 normal opperation, 2 brake all
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

    m_velocityCoefficent = getVelocityCoefficent();

    /*
     * System.out.println("Left Velocity:" + leftSpeed * m_velocityCoefficent *
     * getDistancePerStep() + " Right Velocity:" + rightSpeed * m_velocityCoefficent
     * * getDistancePerStep());
     */

    switch (brakeSide) {
      case "LeftBrake":
        leftSpeed = 0;
        break;
      case "RightBrake":
        rightSpeed = 0;
        break;
      case "FullBrake":
        rightSpeed = 0;
        leftSpeed = 0;
        break;
      case "LeftRadial":
        System.out.println("leftSpeed: " + leftSpeed + " rightSpeed: " + rightSpeed);
        leftSpeed = rightSpeed * m_handlingValues.getRadialTurnCoefficent();
        break;
      case "RightRadial":
        rightSpeed = leftSpeed * m_handlingValues.getRadialTurnCoefficent();
        break;
      case "Normal":
      default:
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

  public void configureTalons() {
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
    return m_leftTalonSensors.getIntegratedSensorPosition(); // average of master and follower?
  }

  public double getRightTalonEncoderPostion() {
    return -m_rightTalonSensors.getIntegratedSensorPosition(); // average of master and follower?
  }

  public double getAverageRobotEncoderVelocity() {
    return (-m_leftTalonSensors.getIntegratedSensorVelocity() + m_rightTalonSensors.getIntegratedSensorVelocity()) / 2;
  }

  public double getAverageAbsoluteRobotEncoderVelocity() {
    return (Math.abs(m_leftTalonSensors.getIntegratedSensorVelocity())
        + Math.abs(m_rightTalonSensors.getIntegratedSensorVelocity())) / 2;
  }

  public void setTargetLeftTalonTargetPosition(double targetSteps) {
    m_leftDriveTalon.set(ControlMode.Position, targetSteps);
  }

  public void setTargetRightTalonTargetPosition(double targetSteps) {
    m_leftDriveTalon.set(ControlMode.Position, targetSteps);
  }

  public double getVelocityCoefficent() // gets the scaling factor to translate the joystick input to the velcoity input
  {
    System.out.println(m_handlingValues.getMaxVelocity());
    return (m_handlingValues.getMaxVelocity() * 0.1 * m_stepsPerRotation / m_gearCoeffiecent / m_wheelCircumference);
    // mps = wheel circumference * gearcoefficent * (steps / 100 ms) * 1000ms /
    // steps per rotation
    // steps / 100ms = mps * .1 seconds * steps per rotation / wheel cicumference /
    // gear ratio
  }

  public double getStepsPerFrameRotation() { // get the number of encoder steps it take the robot to make one full
                                             // rotation
    return m_trackwidth * 6.28 / getDistancePerStep();
  }

  public double getDegreesFrameRotationPerStep() { // get the degrees of robot roation per encoder step
    return 360 * getDistancePerStep() / (6.28 * m_trackwidth);
  }

  public double[] getXYTranslationFromEncoderMovement(double rightMovement, double leftMovement,
      double initialHeading) {

    // becuase I thought this was smart at the time Y is the direction parrellel to
    // the robot, X is perpendiular
    // returns translation in [Y, X, Final Heading]

    double degreesOfRotation = Math.abs(leftMovement - rightMovement) * m_degreesFrameRotationPerStep; // the amount of
                                                                                                       // degrees the
                                                                                                       // robot rotated
    double turnRadius = 28.647 * (leftMovement + rightMovement) / degreesOfRotation;
    // 360 / degreesOfRotation* (leftMovement + rightMovement) / 2 / 2 / 3.14; //
    // the radius of the turn made by the robot
    double xRelativeMovement = turnRadius - Math.cos(degreesOfRotation) * turnRadius; // the x translation relative to
                                                                                      // the initial heading of the
                                                                                      // robot
    // x is side to side
    // left is negative, right is positive
    double yRelativeMovement = Math.sin(degreesOfRotation) * turnRadius;
    // y is forward and backward

    if (rightMovement > leftMovement) { // give the x value the proper sign
      // required because of the absolute value in the degrees of rotation calcualtion
      xRelativeMovement = -xRelativeMovement;
    }
    // Rules:
    // x is side to side
    // left is negative, right is positive
    // y is forward and backward
    // y is the direction the robot is initially at
    // x is perpendicular to the robots direction of heading
    // x is 90 degrees clockwise from the inital heading in the positive direction
    // x direction equals initialHeading - 90
    // initial heading of zero is the robots initial direction of travel
    // the rotation of heading is 0 to 360 in a counterclockwise direction
    double yAbsolute = Math.cos(initialHeading) * yRelativeMovement + Math.cos(initialHeading - 90) * xRelativeMovement;
    double xAbsolute = Math.sin(initialHeading) * xRelativeMovement + Math.cos(initialHeading - 90) * yRelativeMovement;
    // I though about this backward -- x is sin and y is cos in this case
    // --FACESMACK
    double finalHeading = initialHeading + degreesOfRotation;

    return new double[] { yAbsolute, xAbsolute, finalHeading };
  }

  public double[] getXYTranslationFromEncoderGyroMovement(double rightMovement, double leftMovement,
      double initialHeading, double finalHeading) {

    // becuase I thought this was smart at the time Y is the direction parellel to
    // the robot, X is perpendiular
    // returns translation in [Y, X]

    double degreesOfRotation = finalHeading - initialHeading; // the amount of
                                                              // degrees the
                                                              // robot rotated
    double turnRadius = 28.647 * (leftMovement + rightMovement) / degreesOfRotation;
    // 360 / degreesOfRotation* (leftMovement + rightMovement) / 2 / 2 / 3.14; //
    // the radius of the turn made by the robot
    double xRelativeMovement = turnRadius - Math.cos(degreesOfRotation) * turnRadius; // the x translation relative to
                                                                                      // the initial heading of the
                                                                                      // robot
    // x is side to side
    // left is negative, right is positive
    double yRelativeMovement = Math.sin(degreesOfRotation) * turnRadius;
    // y is forward and backward

    if (rightMovement > leftMovement) { // give the x value the proper sign
      // required because of the absolute value in the degrees of rotation calcualtion
      xRelativeMovement = -xRelativeMovement;
    }
    // Rules:
    // x is side to side
    // left is negative, right is positive
    // y is forward and backward
    // y is the direction the robot is initially at
    // x is perpendicular to the robots direction of heading
    // x is 90 degrees clockwise from the inital heading in the positive direction
    // x direction equals initialHeading - 90
    // initial heading of zero is the robots initial direction of travel
    // the rotation of heading is 0 to 360 in a counterclockwise direction
    double yAbsolute = Math.cos(initialHeading) * yRelativeMovement + Math.cos(initialHeading - 90) * xRelativeMovement;
    double xAbsolute = Math.sin(initialHeading) * xRelativeMovement + Math.cos(initialHeading - 90) * yRelativeMovement;
    // I though about this backward -- x is sin and y is cos in this case
    // --FACESMACK

    return new double[] { yAbsolute, xAbsolute };
  }

  public double getDistancePerStep() // meters / step
  {
    return m_wheelCircumference * m_gearCoeffiecent / m_stepsPerRotation;
  }

  public double getLeftJoystickY() {
    double y = m_leftLargeJoystick.getY();

    System.out.println(m_handlingValues.getTankFineHandlingMaxVelocity());
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
    return m_smallJoystick.getRawAxis(4);
  }

  public double getSmallJoystickY() {
    return m_smallJoystick.getY();
  }

  public boolean getSmallJoystickLeftRadialTurnButton() { // returns a boolean from the trigger axis
    if (m_smallJoystick.getRawAxis(2) > 0.5) {
      return true;
    }

    return false;
  }

  public boolean getSmallJoystickRightRadialTurnButton() { // returns a boolean from the trigger axis
    if (m_smallJoystick.getRawAxis(3) > 0.5) {
      return true;
    }

    return false;
  }

  public boolean getLeftJoystickTrigger() {
    return m_leftLargeJoystick.getRawButton(1);
  }

  public boolean getRightJoystickTrigger() {
    return m_rightLargeJoystick.getRawButton(1);
  }

  public boolean getLeftJoystickRadialTurnButton() {
    return m_leftLargeJoystick.getRawButton(4);
  }

  public boolean getRightJoystickRadialTurnButton() {
    return m_rightLargeJoystick.getRawButton(5);
  }

  public double getPDPVoltage() {
    return m_PDP.getVoltage();
  }

  public void updateHandlingBase(HandlingBase base) {
    // updates the drive handling characteristics and refreshes talon configs
    m_handlingValues = base;
    configureTalons();
  }

}
