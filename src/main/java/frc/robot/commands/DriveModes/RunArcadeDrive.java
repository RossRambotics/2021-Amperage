// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveModes;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunArcadeDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drive m_drive;

  public RunArcadeDrive(Drive drive) {
    m_drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Switching to manual arcade drive!");

    m_drive.setBrakeModeLeftDriveTalons(true);
    m_drive.setBrakeModeRightDriveTalons(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_drive.getSmallJoystickLeftShoulder() && m_drive.getSmallJoystickRightShoulder()) {
      m_drive.arcadeDrive(m_drive.getSmallJoystickX(), m_drive.getSmallJoystickY(), "FullBrake");
    } else if (m_drive.getSmallJoystickLeftShoulder()) {
      m_drive.arcadeDrive(m_drive.getSmallJoystickX(), m_drive.getSmallJoystickY(), "LeftBrake");
    } else if (m_drive.getSmallJoystickRightShoulder()) {
      m_drive.arcadeDrive(m_drive.getSmallJoystickX(), m_drive.getSmallJoystickY(), "RightBrake");
    } else if (m_drive.getSmallJoystickLeftRadialTurnButton()) { // radial turn left
      m_drive.arcadeDrive(m_drive.getSmallJoystickX(), m_drive.getSmallJoystickY(), "LeftRadial");
    } else if (m_drive.getSmallJoystickRightRadialTurnButton()) { // radial turn right
      m_drive.arcadeDrive(m_drive.getSmallJoystickX(), m_drive.getSmallJoystickY(), "RightRadial");
    } else {
      m_drive.arcadeDrive(m_drive.getSmallJoystickX(), m_drive.getSmallJoystickY(), "Normal");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drive.setBrakeModeLeftDriveTalons(false);
    m_drive.setBrakeModeRightDriveTalons(false);
    if (interrupted) {
      System.out.println("Manual arcade drive interupted!");
    } else {
      System.out.println("Manual arcade drive ending normally.");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
