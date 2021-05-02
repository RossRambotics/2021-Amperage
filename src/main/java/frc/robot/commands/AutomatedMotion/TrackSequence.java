package frc.robot.commands.AutomatedMotion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;

public class TrackSequence extends SequentialCommandGroup {

    public TrackSequence(Drive drive) {
        this.addCommands(new frc.robot.commands.AutomatedMotion.Track(drive));
    }
}
