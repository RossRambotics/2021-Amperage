// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveModes;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunTankDriveHandBrake extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drive m_drive;

  public RunTankDriveHandBrake(Drive drive) {
    m_drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Switching to manual tank drive!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_drive.getrightJoystickTrigger() && m_drive.getLeftJoystickTrigger()) {
      m_drive.tankDrive(0.0, 0.0);
    } else if (m_drive.getLeftJoystickTrigger()) {
      m_drive.tankDrive(0.0, m_drive.getRightJoystickY());
    } else if (m_drive.getrightJoystickTrigger()) {
      m_drive.tankDrive(m_drive.getLeftJoystickY(), 0.0);
    } else {
      m_drive.tankDrive(m_drive.getLeftJoystickY(), m_drive.getRightJoystickY());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("Manual tank drive interupted!");
    } else {
      System.out.println("Manual tank drive ending normally.");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
