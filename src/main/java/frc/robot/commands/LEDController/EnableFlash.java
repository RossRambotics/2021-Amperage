package frc.robot.commands.LEDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.LEDController;

public class EnableFlash extends CommandBase {

    private LEDController m_LEDController = null;

    public EnableFlash(LEDController LEDcontroller) {
        m_LEDController = LEDcontroller;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_LEDController.enableFlash();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
