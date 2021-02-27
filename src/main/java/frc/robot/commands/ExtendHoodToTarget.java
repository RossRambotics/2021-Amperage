// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TheRobot;
import frc.robot.helper.Targetting.ShooterLookUp;
import frc.robot.subsystems.Hood;

public class ExtendHoodToTarget extends CommandBase {
  private Hood m_hood = null;

  /** Creates a new ExtendHood. */
  public ExtendHoodToTarget(Hood hood) {
    m_hood = hood;

    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TheRobot.log("Test.Hood.ExtendHood: Initialize");
    m_hood.extendToTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
