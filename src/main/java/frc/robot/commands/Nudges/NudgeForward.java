package frc.robot.commands.Nudges;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class NudgeForward extends CommandBase {

    Drive m_drive;

    public NudgeForward(Drive drive) {
        m_drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_drive.nudgeForward();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}