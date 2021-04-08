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

        wayPoints.add(new double[] { -0, 0.524, 1 }); // E2
        wayPoints.add(new double[] { -0.762, 1.286, 1 }); // D3
        wayPoints.add(new double[] { -1.524, 2.048, 1 }); // C4
        wayPoints.add(new double[] { -1.524, 5.096, 1 }); // C8
        wayPoints.add(new double[] { -0.762, 5.9, 1 }); // D9'
        wayPoints.add(new double[] { 0, 6.62, 1 }); // E10
        wayPoints.add(new double[] { -0.762, 7.382, 1 }); // D11
        wayPoints.add(new double[] { -1.524, 6.62, 1 }); // C10
        wayPoints.add(new double[] { -0.762, 5.858, 1 }); // D9
        wayPoints.add(new double[] { 0, 5.096, 1 }); // E8
        wayPoints.add(new double[] { 0, 2.048, 1 }); // E4
        wayPoints.add(new double[] { -0.762, 1.286, 1 }); // D3
        wayPoints.add(new double[] { -1.49, 0.524, 1 }); // C2
        wayPoints.add(new double[] { -1.49, -0.238, 1 }); // C1

        return wayPoints;
    }
}
