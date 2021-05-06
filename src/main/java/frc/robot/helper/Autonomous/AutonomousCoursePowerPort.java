package frc.robot.helper.Autonomous;

import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.Shooter;

public class AutonomousCoursePowerPort extends CourseManager {
    public AutonomousCoursePowerPort(Drive m_drive, Shooter shooter, Indexer indexer, Intake intake, Hood hood,
            LEDController LEDcontroller) {
        super(m_drive, shooter, indexer, intake, hood, LEDcontroller);
    }

    @Override
    protected List<double[]> getWayPointPoints() {
        List<double[]> wayPoints = new ArrayList();
        // its [x, y, trackAfter, shootAfter, lookAhead]

        wayPoints.add(new double[] { 0, -.6, 0, 1, 1 }); // back up slightly and shoot
        wayPoints.add(new double[] { -0.2, -.8, 3, 0, 1 }); // back up and rotate slightly then track three new
                                                            // powercells
        wayPoints.add(new double[] { 0, -.6, 0, 1, 1 }); // drive back up to orginal shooting location and shoot

        return wayPoints;
    }
}
