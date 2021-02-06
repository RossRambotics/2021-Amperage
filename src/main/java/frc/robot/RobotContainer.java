// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeReverse;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.ExampleCommand;

import frc.robot.commands.*;
import frc.robot.commands.AutomatedMotion.AutonomousMovementBase;
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    m_smallJoystick = new Joystick(2); // leave as 2

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

    JoystickButton bButton = new JoystickButton(m_smallJoystick, 2);
    bButton.whenHeld(new frc.robot.commands.IntakeReverse(intake), true);

    JoystickButton rbButton = new JoystickButton(m_smallJoystick, 6);
    rbButton.whenPressed(new frc.robot.commands.IntakeExtend(intake), true);
    rbButton.whenPressed(new frc.robot.commands.IntakeMotorOn(intake), true);

    JoystickButton lbButton = new JoystickButton(m_smallJoystick, 5);
    lbButton.whenPressed(new frc.robot.commands.IntakeRetract(intake), true);
    lbButton.whenPressed(new frc.robot.commands.IntakeMotorOff(intake), true);

    JoystickButton yButton = new JoystickButton(m_smallJoystick, 4);

    JoystickButton xButton = new JoystickButton(m_smallJoystick, 3);
    xButton.whileHeld(new frc.robot.commands.AutomatedMotion.ManualDriveStraight(drive, 1));

    JoystickButton aButton = new JoystickButton(m_smallJoystick, 1);

    JoystickButton selectButton = new JoystickButton(m_smallJoystick, 7);

    JoystickButton startButton = new JoystickButton(m_smallJoystick, 8);
  }

}
