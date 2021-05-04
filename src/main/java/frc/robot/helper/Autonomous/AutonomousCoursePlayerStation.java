package frc.robot.helper.Autonomous;

import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class AutonomousCoursePlayerStation extends CourseManager {
    public AutonomousCoursePlayerStation(Drive m_drive, Shooter shooter, Indexer indexer, Hood hood) {
        super(m_drive, shooter, indexer, hood);
    }

    @Override
    protected List<double[]> getWayPointPoints() {
        List<double[]> wayPoints = new ArrayList();
        // its [x, y, trackAfter, shootAfter, lookAhead]

        wayPoints.add(new double[] { -3, 1, 0, 1, 1 }); // drive forward and towards the power point then shoot\\
        wayPoints.add(new double[] { 2, -1, 2, 0, 1 }); // drive backward and collect the power cells near the control
                                                        // pannel

        return wayPoints;
    }
}
