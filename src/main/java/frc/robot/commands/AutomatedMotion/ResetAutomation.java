// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedMotion;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.TheRobot;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetAutomation extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drive m_drive;

    public ResetAutomation(Drive drive) {
        m_drive = drive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Resetting Absoute Positioning and Automation!");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.resetAbsolultePositioning();
        m_drive.resetGyroYaw();

        if (TheRobot.getInstance().m_robotContainer.resetCourseManager()
                && Robot.m_trackMotionCommand.resetPosition()) {
            System.out.println("Sucessful reset!");
        } else {
            System.out.println("Failed to reset the course!");
        }
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
