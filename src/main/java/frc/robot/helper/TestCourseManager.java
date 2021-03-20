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
        wayPoints.add(new double[] { 0.073664, 0.018416 });
        wayPoints.add(new double[] { 0.09208, 0.11049599999999998 });
        wayPoints.add(new double[] { 0.31307199999999996, 0.8103039999999999 });
        wayPoints.add(new double[] { 0.36832, 0.8839679999999999 });
        wayPoints.add(new double[] { 0.7734719999999999, 1.47328 });
        wayPoints.add(new double[] { 0.9760479999999999, 1.6206079999999998 });
        wayPoints.add(new double[] { 1.418032, 2.1178399999999997 });
        wayPoints.add(new double[] { 1.47328, 2.20992 });
        wayPoints.add(new double[] { 1.7495199999999997, 2.799232 });
        wayPoints.add(new double[] { 1.786352, 2.9281439999999996 });
        wayPoints.add(new double[] { 1.786352, 3.7568639999999998 });
        wayPoints.add(new double[] { 1.786352, 3.8489439999999995 });
        wayPoints.add(new double[] { 1.786352, 4.493504 });
        wayPoints.add(new double[] { 1.786352, 4.585584 });
        wayPoints.add(new double[] { 1.491696, 5.414304 });
        wayPoints.add(new double[] { 1.418032, 5.5248 });
        wayPoints.add(new double[] { 0.902384, 6.1141119999999995 });
        wayPoints.add(new double[] { 0.7918879999999999, 6.150943999999999 });
        wayPoints.add(new double[] { 0.147328, 6.519264 });
        wayPoints.add(new double[] { 0.036832, 6.611343999999999 });
        wayPoints.add(new double[] { 0.257824, 7.311151999999999 });
        wayPoints.add(new double[] { 0.33148799999999995, 7.3664 });
        wayPoints.add(new double[] { 1.068128, 7.550559999999999 });
        wayPoints.add(new double[] { 1.141792, 7.550559999999999 });
        wayPoints.add(new double[] { 1.3443679999999998, 7.532144 });
        wayPoints.add(new double[] { 1.7495199999999997, 7.090159999999999 });
        wayPoints.add(new double[] { 1.7679359999999997, 6.924415999999999 });
        wayPoints.add(new double[] { 1.3812, 6.408767999999999 });
        wayPoints.add(new double[] { 1.2338719999999999, 6.2614399999999995 });
        wayPoints.add(new double[] { 0.6445599999999999, 5.856287999999999 });
        wayPoints.add(new double[] { 0.570896, 5.8010399999999995 });
        wayPoints.add(new double[] { 0.11049599999999998, 5.193312 });
        wayPoints.add(new double[] { 0.073664, 5.082815999999999 });
        wayPoints.add(new double[] { -0.018416, 4.254096 });
        wayPoints.add(new double[] { -0.0, 4.180432 });
        wayPoints.add(new double[] { 0.073664, 3.5174559999999997 });
        wayPoints.add(new double[] { 0.073664, 3.4253759999999995 });
        wayPoints.add(new double[] { 0.11049599999999998, 2.7255679999999995 });
        wayPoints.add(new double[] { 0.11049599999999998, 2.633488 });
        wayPoints.add(new double[] { 0.36832, 2.007344 });
        wayPoints.add(new double[] { 0.38673599999999997, 1.9336799999999998 });
        wayPoints.add(new double[] { 0.9944639999999999, 1.418032 });
        wayPoints.add(new double[] { 1.086544, 1.3259519999999998 });
        wayPoints.add(new double[] { 1.491696, 0.8471359999999999 });
        wayPoints.add(new double[] { 1.5837759999999999, 0.73664 });
        wayPoints.add(new double[] { 1.5837759999999999, -0.128912 });

        return wayPoints;
    }
}
