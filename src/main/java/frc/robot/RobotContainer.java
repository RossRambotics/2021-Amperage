// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeReverse;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.AutomatedMotion.*;

import frc.robot.commands.*;
import frc.robot.commands.AutomatedMotion.AutonomousMovementBase;
import frc.robot.commands.AutomatedMotion.AutonomousMovementBaseII;
import frc.robot.commands.AutomatedMotion.GoToPoint;
import frc.robot.commands.AutomatedMotion.ManualDriveStraight;
import frc.robot.commands.AutomatedMotion.ManualDriveStraightBoosted;
import frc.robot.commands.AutomatedMotion.Track;
import frc.robot.commands.AutomatedMotion.TrackMotionGyro;
import frc.robot.helper.CourseManager;
import frc.robot.helper.TestCourseManager;
import frc.robot.helper.WayPoint;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Robot;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public Joystick m_smallJoystick;
  private Joystick m_leftLargeJoystick = null;
  private Joystick m_rightLargeJoystick = null;
  private JoystickButton m_leftStickButton = null;
  private JoystickButton m_selectButton = null;
  static public CourseManager m_testCourse = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    m_smallJoystick = new Joystick(2); // leave as 2
    m_leftLargeJoystick = new Joystick(1);
    m_rightLargeJoystick = new Joystick(0);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Intake intake = TheRobot.getInstance().m_intake;
    Drive drive = TheRobot.getInstance().m_drive;
    Indexer indexer = TheRobot.getInstance().m_indexer;
    Shooter shooter = TheRobot.getInstance().m_shooter;
    Hood hood = TheRobot.getInstance().m_hood;

    m_testCourse = new TestCourseManager(drive);

    m_leftStickButton = new JoystickButton(m_smallJoystick, 9);
    m_selectButton = new JoystickButton(m_smallJoystick, 7);

    JoystickButton bButton = new JoystickButton(m_smallJoystick, 2);
    bButton.whenHeld(new frc.robot.commands.Indexer.UnloadIndexer(indexer, intake), true);

    JoystickButton yButton = new JoystickButton(m_smallJoystick, 4);
    yButton.whenPressed(new frc.robot.commands.IntakeRetract(intake), true);
    yButton.whenPressed(new frc.robot.commands.IntakeMotorOff(intake), true);

    CommandBase cmd = new SequentialCommandGroup(new frc.robot.commands.ExtendHoodToTarget(hood),
        new ParallelCommandGroup(new frc.robot.commands.StartShooterTargeting(shooter),
            new frc.robot.commands.Target(drive)),
        new frc.robot.commands.Test.Indexer.RunIndexer(indexer).withTimeout(3),
        new frc.robot.commands.Test.Shooter.StopShooter(shooter));

    JoystickButton m_shootButton = new JoystickButton(m_leftLargeJoystick, 5);
    CommandBase shootCommand = new SequentialCommandGroup(new frc.robot.commands.ExtendHoodToTarget(hood),
        new ParallelCommandGroup(new frc.robot.commands.StartShooterTargeting(shooter),
            new frc.robot.commands.Target(drive)),
        new frc.robot.commands.Test.Indexer.RunIndexer(indexer).withTimeout(3),
        new frc.robot.commands.Test.Shooter.StopShooter(shooter));
    m_shootButton.whenPressed(shootCommand, true);

    JoystickButton m_indexButton = new JoystickButton(m_rightLargeJoystick, 4);
    m_indexButton.whenPressed(new frc.robot.commands.Test.Indexer.RunIndexer(indexer).withTimeout(3), true);

    JoystickButton xButton = new JoystickButton(m_smallJoystick, 3);
    xButton.whenPressed(cmd);

    JoystickButton aButton = new JoystickButton(m_smallJoystick, 1);
    aButton.whenPressed(new frc.robot.commands.IntakeExtend(intake), true);
    aButton.whenPressed(new frc.robot.commands.IntakeMotorOn(intake), true);

    JoystickButton startButton = new JoystickButton(m_smallJoystick, 8);
    /*
     * CommandBase track = new SequentialCommandGroup(new Track(drive), new
     * AutonomousMovementBaseII(drive, -.5), new Track(drive), new
     * AutonomousMovementBaseII(drive, -.5), new Track(drive), new
     * AutonomousMovementBaseII(drive, -.4), new AutonomousMovementBaseII(drive,
     * -.4));
     */
    List<WayPoint> firstPoint = new ArrayList();
    List<WayPoint> secondPoint = new ArrayList();
    List<WayPoint> second2Point = new ArrayList();
    List<WayPoint> second3Point = new ArrayList();
    List<WayPoint> thirdPoint = new ArrayList();
    List<WayPoint> fourthPoint = new ArrayList();
    List<WayPoint> fifthPoint = new ArrayList();
    List<WayPoint> sixthPoint = new ArrayList();
    List<WayPoint> endZone = new ArrayList();

    firstPoint.add(new WayPoint(-.55, -1.824));
    secondPoint.add(new WayPoint(-.55, -2.1));
    second2Point.add(new WayPoint(0, -2.4));
    second3Point.add(new WayPoint(.762, -3.3));
    thirdPoint.add(new WayPoint(.762, -3.7));
    fourthPoint.add(new WayPoint(-.2, -4.1));
    fifthPoint.add(new WayPoint(-.762, -4.8));
    sixthPoint.add(new WayPoint(.01, -5.4));
    endZone.add(new WayPoint(.05, -9.92));

    CommandBase track = new SequentialCommandGroup(new GoToPoint(drive, firstPoint), new GoToPoint(drive, secondPoint),
        new GoToPoint(drive, second2Point), new GoToPoint(drive, second3Point), new GoToPoint(drive, thirdPoint),
        new GoToPoint(drive, fourthPoint), new GoToPoint(drive, fifthPoint), new GoToPoint(drive, sixthPoint),
        new GoToPoint(drive, endZone));
    startButton.whenPressed(track);

    m_selectButton.whenPressed(m_testCourse.getCourseCommand());

    m_leftStickButton.whenPressed(new frc.robot.commands.AutomatedMotion.ResetAutomation(drive));

    if (m_leftLargeJoystick != null) {
      JoystickButton leftTopForwardButton = new JoystickButton(m_leftLargeJoystick, 3);
      leftTopForwardButton.whileHeld(new ManualDriveStraight(drive, 1));

      JoystickButton leftTopBottomButton = new JoystickButton(m_leftLargeJoystick, 2);
      leftTopBottomButton.whileHeld(new ManualDriveStraightBoosted(drive, 1));
    }

    if (m_rightLargeJoystick != null) {
      JoystickButton rightTopForwardButton = new JoystickButton(m_rightLargeJoystick, 3);
      rightTopForwardButton.whileHeld(new ManualDriveStraight(drive, 0));

      JoystickButton rightTopBottomButton = new JoystickButton(m_rightLargeJoystick, 2);
      rightTopBottomButton.whileHeld(new ManualDriveStraightBoosted(drive, 0));
    }

  }

  public boolean resetCourseManager() {
    if (m_testCourse != null) {
      m_testCourse.resetCourse();
      m_selectButton.whenPressed(m_testCourse.getCourseCommand());
      return true;
    }

    return false;
  }
}
