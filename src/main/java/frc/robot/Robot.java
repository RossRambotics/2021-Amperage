// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helper.DriveHandlingSetup.DefaultHardSurfaceArcadeDrive;
import frc.robot.helper.DriveHandlingSetup.DefaultHardSurfaceHandling;
import frc.robot.helper.DriveHandlingSetup.HandlingBase;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public Drive m_drive = null;
  public Indexer m_indexer = null;
  public Intake m_intake = null;
  public Shooter m_shooter = null;
  public Hood m_hood = null;
  private HandlingBase m_handlingBase;

  public Robot() {
    super();
    TheRobot.SetInstance(this);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    TheRobot.log("robotInit.");

    m_handlingBase = new DefaultHardSurfaceArcadeDrive(); // change out handling base to set default handling
    // other handling modes avialble in shuffleboard in drive tab

    // m_climber = new Climber();
    // m_controlPanel = new ControlPanel();
    m_drive = new Drive(m_handlingBase);
    m_drive.setDefaultCommand(m_handlingBase.getDefaultDriveCommand(m_drive)); // sets the default drive command

    m_intake = new Intake();
    m_indexer = new Indexer();
    m_shooter = new Shooter();
    m_hood = new Hood();
    // m_PCTargeter = new PowerCellTargeter();
    // m_LEDs = new LEDController();

    m_robotContainer = new RobotContainer();
    m_intake.createShuffleBoardTab();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
