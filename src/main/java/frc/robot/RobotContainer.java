// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Intake;

import frc.robot.commands.AutomatedMotion.ManualDriveStraight;
import frc.robot.commands.AutomatedMotion.ManualDriveStraightBoosted;
import frc.robot.commands.Shoot.StopShooter;
import frc.robot.helper.Autonomous.CourseManager;
import frc.robot.helper.Autonomous.TestCourseManager;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public Joystick m_smallDriverJoystick = null;
  private Joystick m_leftLargeJoystick = null;
  private Joystick m_rightLargeJoystick = null;
  private Joystick m_smallOperatorJoystick = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    m_smallDriverJoystick = new Joystick(2); // leave as 2
    m_smallOperatorJoystick = new Joystick(3); // leave as 3
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
    Climb climb = TheRobot.getInstance().m_climb;
    LEDController LEDcontroller = TheRobot.getInstance().m_LEDController;

    configureOperatorButtons(m_smallOperatorJoystick, drive, indexer, intake, climb);
    configureTankDriverButtons(m_leftLargeJoystick, 1, drive, shooter, indexer, hood, LEDcontroller);
    configureTankDriverButtons(m_rightLargeJoystick, 0, drive, shooter, indexer, hood, LEDcontroller);

  }

  private void configureTankDriverButtons(Joystick joystick, int stickNumber, Drive drive, Shooter shooter,
      Indexer indexer, Hood hood, LEDController LEDcontroller) {
    // leftstick 1, rightstick 0

    // shoot sequence
    JoystickButton topCenterButton = new JoystickButton(joystick, 3);
    topCenterButton
        .whenPressed(new frc.robot.commands.Shoot.StandingShootSequence(drive, shooter, hood, indexer, LEDcontroller));

    // drive straight fast
    JoystickButton bottomCenterButton = new JoystickButton(joystick, 2);
    bottomCenterButton.whileHeld(new ManualDriveStraightBoosted(drive, stickNumber));

    if (stickNumber == 1) {
      JoystickButton upperInsideButton = new JoystickButton(joystick, 5);
      upperInsideButton.whenPressed(new StopShooter(shooter));
    }
  }

  private void configureOperatorButtons(Joystick joystick, Drive drive, Indexer indexer, Intake intake, Climb climb) {
    // Deploy and start intake
    JoystickButton aButton = new JoystickButton(joystick, 1);
    aButton.whenPressed(new ParallelCommandGroup(new frc.robot.commands.Intake.IntakeExtend(intake),
        new frc.robot.commands.Intake.IntakeMotorOn(intake)));

    // Reverse intake
    JoystickButton bButton = new JoystickButton(joystick, 2);
    bButton.whenPressed(new frc.robot.commands.Intake.IntakeReverse(intake));

    // inhale and stop intake
    JoystickButton yButton = new JoystickButton(joystick, 4);
    yButton.whenPressed(new ParallelCommandGroup(new frc.robot.commands.Intake.IntakeRetract(intake),
        new SequentialCommandGroup(new WaitCommand(0.1), new frc.robot.commands.Intake.IntakeMotorOff(intake))));

    // stop intake
    JoystickButton xButton = new JoystickButton(joystick, 3);
    xButton.whenPressed(new frc.robot.commands.Intake.IntakeMotorOff(intake));

    // reverse indexer and intake
    JoystickButton backButton = new JoystickButton(joystick, 8);
    backButton.whileHeld(new frc.robot.commands.Indexer.UnloadIndexer(indexer, intake));

    // move the indexer forward
    JoystickButton selectButton = new JoystickButton(joystick, 7);
    selectButton.whileHeld(new frc.robot.commands.Indexer.RunIndexer(indexer));

    // retract left winch
    JoystickButton leftShoulder = new JoystickButton(joystick, 5);
    leftShoulder.whileHeld(new frc.robot.commands.Climb.RetractLeftWinch(climb));

    // retract right winch
    JoystickButton rightShoulder = new JoystickButton(joystick, 6);
    rightShoulder.whileHeld(new frc.robot.commands.Climb.RetractRightWinch(climb));

    // nudge left
    POVButton leftPOV = new POVButton(joystick, 270);
    leftPOV.whenPressed(new frc.robot.commands.Nudges.NudgeCounterClockwise(drive).withTimeout(0.3));

    // nudge right
    POVButton rightPOV = new POVButton(joystick, 90);
    rightPOV.whenPressed(new frc.robot.commands.Nudges.NudgeClockwise(drive).withTimeout(0.3));

    // nudge backward
    POVButton backwardPOV = new POVButton(joystick, 180);
    backwardPOV.whenPressed(new frc.robot.commands.Nudges.NudgeBackward(drive).withTimeout(0.3));

    // nudge forward
    POVButton forwardPOV = new POVButton(joystick, 0);
    forwardPOV.whenPressed(new frc.robot.commands.Nudges.NudgeForward(drive).withTimeout(0.3));
  }

  private void configureArcadeDriverButtons(Joystick joystick, Indexer indexer, Intake intake) {
    // configure intake extend and start
    JoystickButton aButton = new JoystickButton(joystick, 1);
    aButton.whenPressed(new frc.robot.commands.Intake.IntakeExtend(intake), true);
    aButton.whenPressed(new frc.robot.commands.Intake.IntakeMotorOn(intake), true);

    // configure intake stop and retract
    JoystickButton bButton = new JoystickButton(joystick, 2);
    bButton.whenPressed(new frc.robot.commands.Intake.IntakeRetract(intake), true);
    bButton.whenPressed(new frc.robot.commands.Intake.IntakeMotorOff(intake), true);

    // configure reverse function
    JoystickButton leftStickButton = new JoystickButton(joystick, 9);
    leftStickButton.whenHeld(new frc.robot.commands.Indexer.UnloadIndexer(indexer, intake), true);
  }
}
