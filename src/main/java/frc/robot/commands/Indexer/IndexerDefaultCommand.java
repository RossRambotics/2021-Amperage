package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IndexerDefaultCommand extends CommandBase {

    private Indexer m_indexer = null;
    private Intake m_intake = null;

    /** Creates a new CheckPowercell. */
    public IndexerDefaultCommand(Indexer indexer, Intake intake) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_indexer = indexer;
        m_intake = intake;
        this.addRequirements(m_indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_indexer.getShooterIndexerSensor()) {
            // back up indexer to prevent shooter failure
            m_indexer.reverse();

            if (m_indexer.getIndexerFrontSensor()) {
                // disable the feeder wheels to prevent overcrowding
                m_intake.disableFeederWheels();
                m_intake.stopFeederWheels();
                m_indexer.setIndexerAdvanceEnable(false);
            }
        } else if (m_indexer.getIntakeIndexerSensor() && m_indexer.getIndexerAdvanceEnabled()) {
            // index the power cell and enable the compact sequence
            m_indexer.advance();
            m_indexer.enableCompact(); // resets timer for compact
        } else if (m_indexer.getIndexerRearSensor() && m_indexer.getIndexerAdvanceEnabled()) {
            m_indexer.advance();
        } else if (m_indexer.getIndexerFrontSensor()) {
            m_indexer.stop();
        } else {
            m_indexer.compact();
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
