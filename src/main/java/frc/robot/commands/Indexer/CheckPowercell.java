// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;

public class CheckPowercell extends CommandBase {
  private Indexer m_indexer = null;

  /** Creates a new CheckPowercell. */
  public CheckPowercell(Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_indexer = indexer;
    this.addRequirements(m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_indexer.checkIndexerSensor() && !m_indexer.checkFrontTopLight()) {
      // CommandBase c = new RunFeeder(m_indexer);
      CommandBase c = new SequentialCommandGroup(new InsertPowercell(m_indexer));
      CommandScheduler.getInstance().schedule(c);
    } else if (m_indexer.checkFrontTopLight()) {
      System.out.println("TopBackLight");

      m_indexer.reverse();

    } else {
      m_indexer.stop();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
