package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class RetractRightWinch extends CommandBase {

    private Climb m_climb = null;

    public RetractRightWinch(Climb climb) {
        m_climb = climb;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_climb.rightWinchRetract();
    }

    @Override
    public void end(boolean interrupted) {
        m_climb.rightWinchStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
