package frc.robot.commands.LEDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.LEDController;

public class DisableFlash extends CommandBase {

    private LEDController m_LEDController = null;

    public DisableFlash(LEDController LEDcontroller) {
        m_LEDController = LEDcontroller;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_LEDController.disableFlash();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
