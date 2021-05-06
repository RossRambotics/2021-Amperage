package frc.robot.helper.Autonomous;

import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.Shooter;

public class AutonomousCourseCenter extends CourseManager {
    public AutonomousCourseCenter(Drive m_drive, Shooter shooter, Indexer indexer, Hood hood,
            LEDController LEDcontroller) {
        super(m_drive, shooter, indexer, hood, LEDcontroller);
    }

    @Override
    protected List<double[]> getWayPointPoints() {
        List<double[]> wayPoints = new ArrayList();
        // its [x, y, trackAfter, shootAfter, lookAhead]

        wayPoints.add(new double[] { -1, 1, 0, 1, 1 }); // drive forward and towards the power point then shoot
        wayPoints.add(new double[] { 0, -1, 3, 0, 1 }); // drive backward and towards the power cells under the sheild
                                                        // generator
        wayPoints.add(new double[] { -1, 1, 0, 1, 1 }); // drive forward and towards the power cell and shoot

        return wayPoints;
    }
}
