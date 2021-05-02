package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

// This class is the shoot sequence for a target and shoot at rest

public class StandingShootSequence extends SequentialCommandGroup {

    public StandingShootSequence(Drive drive, Shooter shooter, Hood hood, Indexer indexer) {
        this.addCommands(new ParallelCommandGroup(new frc.robot.commands.Shoot.StartShooterTargeting(shooter),
                new frc.robot.commands.Shoot.Target(drive), new frc.robot.commands.Shoot.ExtendHoodToTarget(hood)),
                new frc.robot.commands.Indexer.RunIndexer(indexer).withTimeout(3),
                new frc.robot.commands.Shoot.StopShooter(shooter));
    }
}
