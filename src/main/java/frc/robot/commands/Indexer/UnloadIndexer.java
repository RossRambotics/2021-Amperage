// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class UnloadIndexer extends CommandBase {
  private Indexer m_indexer = null;
  private Intake m_intake = null;

  /** Creates a new UnloadIndexer. */
  public UnloadIndexer(Indexer indexer, Intake intake) {
    m_indexer = indexer;
    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(indexer, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexer.reverse();
    m_intake.IntakeMotorReverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stop();
    m_intake.IntakeMotorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}