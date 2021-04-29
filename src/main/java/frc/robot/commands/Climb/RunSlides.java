package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class RunSlides extends CommandBase {

    private Climb m_climb;

    public RunSlides(Climb climb) {
        addRequirements(climb);

        m_climb = climb;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_climb.setLeftSlide(m_climb.m_smallOperatorJoystick.getRawAxis(1));
        m_climb.setRightSlide(m_climb.m_smallOperatorJoystick.getRawAxis(5));

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
