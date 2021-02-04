package frc.robot.commands.AutomatedMotion;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class AutonomousMovementBase extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drive m_drive;

    private Double m_targetMeters;
    private Double m_targetSteps; // the intended amount of steps
    private Double m_targetMaxVelocity; // peak velocity in mps

    public AutonomousMovementBase(Drive drive, Double targetMeters, Double targetMaxVelocity) {
      m_drive = drive;
      m_targetMeters = targetMeters;
      m_targetSteps = m_drive.getDistancePerStep() * targetMeters;
      m_targetMaxVelocity = targetMaxVelocity;

      addRequirements(drive);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }