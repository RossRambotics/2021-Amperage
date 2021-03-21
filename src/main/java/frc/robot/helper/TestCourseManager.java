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
        wayPoints.add(new double[] { 0.11049599999999998, 0.9944639999999999 });
        wayPoints.add(new double[] { 0.55248, 1.5469439999999999 });
        wayPoints.add(new double[] { 0.7734719999999999, 1.6574399999999998 });
        wayPoints.add(new double[] { 1.10496, 1.8784319999999999 });
        wayPoints.add(new double[] { 1.3259519999999998, 1.9889279999999998 });
        wayPoints.add(new double[] { 1.8784319999999999, 2.7624 });
        wayPoints.add(new double[] { 1.8784319999999999, 3.7568639999999998 });
        wayPoints.add(new double[] { 1.7679359999999997, 4.8618239999999995 });
        wayPoints.add(new double[] { 1.5469439999999999, 5.745792 });
        wayPoints.add(new double[] { 1.2154559999999999, 6.077279999999999 });
        wayPoints.add(new double[] { 1.10496, 6.077279999999999 });
        wayPoints.add(new double[] { 0.33148799999999995, 6.408767999999999 });
        wayPoints.add(new double[] { -0.11049599999999998, 6.629759999999999 });
        wayPoints.add(new double[] { -0.11049599999999998, 6.740256 });
        wayPoints.add(new double[] { -0.11049599999999998, 6.961247999999999 });
        wayPoints.add(new double[] { 0.1104959999, 6.9612479999 });
        wayPoints.add(new double[] { 0.1104959999, 7.071743999900001 });
        wayPoints.add(new double[] { 0.22099199989999999, 7.4032319999 });
        wayPoints.add(new double[] { 0.3314879999, 7.5137279999 });
        wayPoints.add(new double[] { 0.5524799999, 7.7347199999 });
        wayPoints.add(new double[] { 0.7734719998999999, 7.845215999900001 });
        wayPoints.add(new double[] { 0.8839679999, 7.845215999900001 });
        wayPoints.add(new double[] { 1.4364479998999997, 7.7347199999 });
        wayPoints.add(new double[] { 1.6574399998999998, 7.6242239999 });
        wayPoints.add(new double[] { 1.7679359998999997, 7.5137279999 });
        wayPoints.add(new double[] { 1.7679359998999997, 7.4032319999 });
        wayPoints.add(new double[] { 1.6574399998999998, 6.6297599999 });
        wayPoints.add(new double[] { 1.5469439998999999, 6.4087679999 });
        wayPoints.add(new double[] { 1.4364479998999997, 6.298271999900001 });
        wayPoints.add(new double[] { 0.7734719998999999, 5.8562879999 });
        wayPoints.add(new double[] { 0.5524799999, 5.745791999900001 });
        wayPoints.add(new double[] { 0.1104959999, 5.193311999900001 });
        wayPoints.add(new double[] { -9.999998051846148e-11, 4.972319999900001 });
        wayPoints.add(new double[] { -9.999998051846148e-11, 4.8618239999 });
        wayPoints.add(new double[] { -0.11049600009999996, 4.1988479999 });
        wayPoints.add(new double[] { -0.11049600009999996, 3.2043839999000006 });
        wayPoints.add(new double[] { -0.11049600009999996, 3.0938879999000006 });
        wayPoints.add(new double[] { -9.999998051846148e-11, 2.8728959999000008 });
        wayPoints.add(new double[] { -9.999998051846148e-11, 2.7623999999000004 });
        wayPoints.add(new double[] { 0.1104959999, 2.3204159999000007 });
        wayPoints.add(new double[] { 0.22099199989999999, 2.099423999900001 });
        wayPoints.add(new double[] { 0.4419839998999999, 1.7679359999000006 });
        wayPoints.add(new double[] { 0.5524799999, 1.6574399999000011 });
        wayPoints.add(new double[] { 0.7734719998999999, 1.5469439999000008 });
        wayPoints.add(new double[] { 1.3259519998999998, 1.325951999900001 });
        wayPoints.add(new double[] { 1.6574399998999998, 1.2154559999000005 });
        wayPoints.add(new double[] { 1.8784319998999996, 1.104959999900001 });
        wayPoints.add(new double[] { 1.9889279998999998, 0.9944639999000007 });
        wayPoints.add(new double[] { 1.9889279998999998, 0.8839679999000012 });
        wayPoints.add(new double[] { 1.8784319998999996, -9.99991200956174e-11 });

        return wayPoints;
    }
}
