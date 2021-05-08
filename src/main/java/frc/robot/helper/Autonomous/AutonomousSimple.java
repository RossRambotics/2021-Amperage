package frc.robot.helper.Autonomous;

import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.Shooter;

public class AutonomousSimple extends CourseManager {
    public AutonomousSimple(Drive m_drive, Shooter shooter, Indexer indexer, Intake intake, Hood hood,
            LEDController LEDcontroller) {
        super(m_drive, shooter, indexer, intake, hood, LEDcontroller);
    }

    @Override
    protected List<double[]> getWayPointPoints() {
        List<double[]> wayPoints = new ArrayList();
        // its [x, y, trackAfter, shootAfter, lookAhead]

        wayPoints.add(new double[] { 0, -.6, 0, 1, 1 }); // back up slightly and shoot

        return wayPoints;
    }
}
