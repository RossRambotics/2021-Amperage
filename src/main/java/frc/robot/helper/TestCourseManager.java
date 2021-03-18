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
        wayPoints.add(new double[] { -0.0, 0.09208 });
        wayPoints.add(new double[] { -0.0, 0.18416 });
        wayPoints.add(new double[] { -0.0, 0.27624 });
        wayPoints.add(new double[] { -0.0, 0.36832 });
        wayPoints.add(new double[] { -0.0, 0.4604 });
        wayPoints.add(new double[] { -0.018416, 0.534064 });
        wayPoints.add(new double[] { -0.018416, 0.6261439999999999 });
        wayPoints.add(new double[] { -0.018416, 0.718224 });
        wayPoints.add(new double[] { -0.036832, 0.7918879999999999 });
        wayPoints.add(new double[] { -0.05524799999999999, 0.8471359999999999 });
        wayPoints.add(new double[] { -0.073664, 0.9392159999999999 });
        wayPoints.add(new double[] { -0.11049599999999998, 0.9944639999999999 });
        wayPoints.add(new double[] { -0.05524799999999999, 1.086544 });
        wayPoints.add(new double[] { -0.05524799999999999, 1.178624 });
        wayPoints.add(new double[] { -0.05524799999999999, 1.2707039999999998 });
        wayPoints.add(new double[] { -0.05524799999999999, 1.3627839999999998 });
        wayPoints.add(new double[] { -0.036832, 1.454864 });
        wayPoints.add(new double[] { -0.036832, 1.5469439999999999 });
        wayPoints.add(new double[] { -0.036832, 1.6390239999999998 });
        wayPoints.add(new double[] { -0.018416, 1.7126879999999998 });
        wayPoints.add(new double[] { -0.018416, 1.804768 });
        wayPoints.add(new double[] { -0.018416, 1.8968479999999999 });
        wayPoints.add(new double[] { -0.0, 2.02576 });
        wayPoints.add(new double[] { 0.018416, 2.1178399999999997 });
        wayPoints.add(new double[] { 0.05524799999999999, 2.2283359999999997 });
        wayPoints.add(new double[] { 0.11049599999999998, 2.320416 });
        wayPoints.add(new double[] { 0.16574399999999997, 2.412496 });
        wayPoints.add(new double[] { 0.22099199999999997, 2.4677439999999997 });
        wayPoints.add(new double[] { 0.294656, 2.522992 });
        wayPoints.add(new double[] { 0.44198399999999993, 2.5414079999999997 });
        wayPoints.add(new double[] { 0.515648, 2.5782399999999996 });
        wayPoints.add(new double[] { 0.589312, 2.596656 });
        wayPoints.add(new double[] { 0.6629759999999999, 2.6150719999999996 });
        wayPoints.add(new double[] { 0.7918879999999999, 2.633488 });
        wayPoints.add(new double[] { 0.8471359999999999, 2.6150719999999996 });
        wayPoints.add(new double[] { 0.902384, 2.596656 });
        wayPoints.add(new double[] { 0.9576319999999999, 2.559824 });
        wayPoints.add(new double[] { 1.01288, 2.522992 });
        wayPoints.add(new double[] { 1.068128, 2.4677439999999997 });
        wayPoints.add(new double[] { 1.123376, 2.4309119999999997 });
        wayPoints.add(new double[] { 1.160208, 2.357248 });
        wayPoints.add(new double[] { 1.1970399999999999, 2.3019999999999996 });
        wayPoints.add(new double[] { 1.2522879999999998, 2.2651679999999996 });
        wayPoints.add(new double[] { 1.2891199999999998, 2.20992 });
        wayPoints.add(new double[] { 1.3443679999999998, 2.173088 });
        wayPoints.add(new double[] { 1.3812, 2.099424 });
        wayPoints.add(new double[] { 1.399616, 2.02576 });
        wayPoints.add(new double[] { 1.436448, 1.9705119999999998 });
        wayPoints.add(new double[] { 1.436448, 1.8784319999999999 });
        wayPoints.add(new double[] { 1.436448, 1.786352 });
        wayPoints.add(new double[] { 1.454864, 1.7126879999999998 });
        wayPoints.add(new double[] { 1.454864, 1.6206079999999998 });
        wayPoints.add(new double[] { 1.454864, 1.5285279999999999 });
        wayPoints.add(new double[] { 1.418032, 1.436448 });
        wayPoints.add(new double[] { 1.3627839999999998, 1.3259519999999998 });
        wayPoints.add(new double[] { 1.3075359999999998, 1.2338719999999999 });
        wayPoints.add(new double[] { 1.2338719999999999, 1.141792 });
        wayPoints.add(new double[] { 1.160208, 1.049712 });
        wayPoints.add(new double[] { 1.086544, 0.9944639999999999 });
        wayPoints.add(new double[] { 1.01288, 0.9576319999999999 });
        wayPoints.add(new double[] { 0.9392159999999999, 0.9576319999999999 });
        wayPoints.add(new double[] { 0.8655519999999999, 0.9576319999999999 });
        wayPoints.add(new double[] { 0.7918879999999999, 0.9576319999999999 });
        wayPoints.add(new double[] { 0.718224, 0.9576319999999999 });
        wayPoints.add(new double[] { 0.6445599999999999, 0.9576319999999999 });
        wayPoints.add(new double[] { 0.589312, 0.9760479999999999 });
        wayPoints.add(new double[] { 0.515648, 0.9760479999999999 });
        wayPoints.add(new double[] { 0.44198399999999993, 0.9760479999999999 });
        wayPoints.add(new double[] { 0.38673599999999997, 0.9944639999999999 });
        wayPoints.add(new double[] { 0.33148799999999995, 0.9760479999999999 });
        wayPoints.add(new double[] { 0.27624, 0.9944639999999999 });
        wayPoints.add(new double[] { 0.22099199999999997, 0.9760479999999999 });
        wayPoints.add(new double[] { 0.147328, 0.9760479999999999 });
        wayPoints.add(new double[] { 0.073664, 0.9760479999999999 });
        wayPoints.add(new double[] { -0.0, 0.9392159999999999 });
        wayPoints.add(new double[] { -0.16574399999999997, 0.9576319999999999 });
        wayPoints.add(new double[] { -0.23940799999999998, 0.9576319999999999 });
        wayPoints.add(new double[] { -0.31307199999999996, 0.9392159999999999 });
        wayPoints.add(new double[] { -0.38673599999999997, 0.9392159999999999 });
        wayPoints.add(new double[] { -0.4604, 0.9208 });
        wayPoints.add(new double[] { -0.534064, 0.9208 });
        wayPoints.add(new double[] { -0.6077279999999999, 0.9208 });
        wayPoints.add(new double[] { -0.6813919999999999, 0.9208 });

        return wayPoints;
    }
}
