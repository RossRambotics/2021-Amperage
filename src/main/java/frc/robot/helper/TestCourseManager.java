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

        wayPoints.add(new double[] { -0.0, 0.0 });
        wayPoints.add(new double[] { 0.20257599999999998, 0.6445599999999999 });
        wayPoints.add(new double[] { 0.257824, 0.6813919999999999 });
        wayPoints.add(new double[] { 0.31307199999999996, 0.755056 });
        wayPoints.add(new double[] { 0.4604, 0.9208 });
        wayPoints.add(new double[] { 0.534064, 0.9760479999999999 });
        wayPoints.add(new double[] { 0.6077279999999999, 0.9944639999999999 });
        wayPoints.add(new double[] { 0.8287199999999999, 1.031296 });
        wayPoints.add(new double[] { 0.902384, 1.031296 });
        wayPoints.add(new double[] { 1.6942719999999998, 0.8839679999999999 });
        wayPoints.add(new double[] { 1.7311039999999998, 0.8287199999999999 });
        wayPoints.add(new double[] { 1.8968479999999999, 0.515648 });
        wayPoints.add(new double[] { 1.9152639999999999, 0.44198399999999993 });
        wayPoints.add(new double[] { 1.9889279999999998, -0.036832 });
        wayPoints.add(new double[] { 2.007344, -0.11049599999999998 });
        wayPoints.add(new double[] { 1.9705119999999998, -0.294656 });
        wayPoints.add(new double[] { 1.9336799999999998, -0.38673599999999997 });
        wayPoints.add(new double[] { 1.823184, -0.589312 });
        wayPoints.add(new double[] { 1.5469439999999999, -0.8655519999999999 });
        wayPoints.add(new double[] { 1.47328, -0.902384 });
        wayPoints.add(new double[] { 1.10496, -0.9760479999999999 });
        wayPoints.add(new double[] { 0.6629759999999999, -1.031296 });
        wayPoints.add(new double[] { 0.589312, -1.049712 });
        wayPoints.add(new double[] { 0.515648, -1.086544 });
        wayPoints.add(new double[] { 0.018416, -1.9705119999999998 });
        wayPoints.add(new double[] { -0.0, -3.4437919999999997 });
        wayPoints.add(new double[] { 0.073664, -4.014688 });
        wayPoints.add(new double[] { 0.09208, -4.0883519999999995 });
        wayPoints.add(new double[] { 0.22099199999999997, -4.640832 });
        wayPoints.add(new double[] { 0.31307199999999996, -4.935487999999999 });
        wayPoints.add(new double[] { 0.36832, -4.97232 });
        wayPoints.add(new double[] { 0.49723199999999995, -5.138064 });
        wayPoints.add(new double[] { 0.55248, -5.1748959999999995 });
        wayPoints.add(new double[] { 0.6629759999999999, -5.2485599999999994 });
        wayPoints.add(new double[] { 0.718224, -5.285392 });
        wayPoints.add(new double[] { 0.902384, -5.359056 });
        wayPoints.add(new double[] { 0.9576319999999999, -5.377471999999999 });
        wayPoints.add(new double[] { 1.1970399999999999, -5.469551999999999 });
        wayPoints.add(new double[] { 1.491696, -5.598464 });
        wayPoints.add(new double[] { 1.5469439999999999, -5.635295999999999 });
        wayPoints.add(new double[] { 1.804768, -5.856287999999999 });
        wayPoints.add(new double[] { 1.860016, -6.869167999999999 });

        return wayPoints;
    }
}
