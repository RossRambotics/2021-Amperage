// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TheRobot;
import frc.robot.subsystems.Intake;

public class IntakeReverse extends CommandBase {

  public Intake intake;

  /** Creates a new IntakeReverse. */
  public IntakeReverse(Intake intake1) {
    intake = intake1;

    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(intake1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TheRobot.log("In IntakeReverse command:  Initialize!");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TheRobot.log("In IntakeReverse command:  Executing!");
    intake.IntakeMotorReverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.capture();
    TheRobot.log("IntakeReverse Ended.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}