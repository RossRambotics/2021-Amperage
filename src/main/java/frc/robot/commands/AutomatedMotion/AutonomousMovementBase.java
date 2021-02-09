package frc.robot.commands.AutomatedMotion;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class AutonomousMovementBase extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drive m_drive;

  private double m_targetMeters;
  private double m_targetSteps; // the intended amount of steps
  private double m_targetMaxVelocity; // peak velocity in mps
  private double m_intialStepsLeft;
  private double m_intialStepsRight; // the inital step count when the command begins

  public AutonomousMovementBase(Drive drive, double targetMeters, double targetMaxVelocity) {
    m_drive = drive;
    m_targetMeters = targetMeters;
    m_targetSteps = m_drive.getDistancePerStep() * targetMeters;
    m_targetMaxVelocity = targetMaxVelocity;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    return false;
  }
}