// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeMotorOff;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class CheckPowercell extends CommandBase {
  private Indexer m_indexer = null;
  private Intake m_intake = null;

  /** Creates a new CheckPowercell. */
  public CheckPowercell(Indexer indexer, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_indexer = indexer;
    m_intake = intake;
    this.addRequirements(m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_indexer.checkIndexerSensor() == true) {
      // CommandBase c = new RunFeeder(m_indexer);
      CommandBase c = new SequentialCommandGroup(new InsertPowercell(m_indexer));
      CommandScheduler.getInstance().schedule(c);
    }
    if (m_indexer.checkBackTopLight() == true && m_indexer.checkFrontTopLight() == true) {
      m_indexer.stop();
      CommandScheduler.getInstance().schedule(new IntakeMotorOff(m_intake));
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
