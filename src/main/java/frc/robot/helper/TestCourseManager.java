package frc.robot.helper;

import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.Drive;

public class TestCourseManager extends CourseManager {
    public TestCourseManager(Drive m_drive) {
        super(m_drive);
    }

    @Override
    protected List<double[]> getWayPointPoints() {
        List<double[]> wayPoints = new ArrayList();

        wayPoints.add(new double[] { 1, 1 });
        wayPoints.add(new double[] { 1, -1 });
        wayPoints.add(new double[] { -1, -1 });
        wayPoints.add(new double[] { -1, 1 });
        wayPoints.add(new double[] { 0, 0 });

        return wayPoints;
    }
}
