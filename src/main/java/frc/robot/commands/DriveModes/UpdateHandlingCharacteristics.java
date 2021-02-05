package frc.robot.commands.DriveModes;

import frc.robot.helper.DriveHandlingSetup.HandlingBase;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class UpdateHandlingCharacteristics extends CommandBase {
  private final Drive m_drive;
  private HandlingBase m_handlingBase;

  public UpdateHandlingCharacteristics(Drive drive, HandlingBase base) {
    m_drive = drive;
    m_handlingBase = base;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().cancel(CommandScheduler.getInstance().getDefaultCommand(m_drive));
    m_drive.setDefaultCommand(m_handlingBase.getDefaultDriveCommand(m_drive)); // updates the default drive mode
    m_drive.updateHandlingBase(m_handlingBase);
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
