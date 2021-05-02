package frc.robot.helper.Autonomous;

import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Stack;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutomatedMotion.GoToPoint;
import frc.robot.subsystems.Drive;

public class CourseManager {

    private Stack<List<WayPoint>> wayPoints;
    private Drive m_drive;
    private double m_lookAheadValue = 5;

    public CourseManager(Drive drive) {
        wayPoints = new Stack();
        m_drive = drive;

        fillWayPointStack();
    }

    public void resetCourse() { // reset with the new cleared points
        wayPoints = new Stack();
        fillWayPointStack();
    }

    private void fillWayPointStack() {
        List<double[]> points = getWayPointPoints();

        for (int i = points.size() - 1; i >= 0; i = i - 1) { // for loop flips order of points
            List<WayPoint> lookAheadList = new ArrayList<>();

            m_lookAheadValue = points.get(i)[2];
            for (int f = i; f < i + m_lookAheadValue && f < points.size(); f = f + 1) {
                lookAheadList.add(new WayPoint(-points.get(f)[0], points.get(f)[1]));
            }

            wayPoints.push(lookAheadList);
        }
    }

    protected List<double[]> getWayPointPoints() {
        List<double[]> points = new ArrayList<>();

        points.add(new double[] { 0, 0 }); // override and add points like this
        // its [x, y]
        // add them in order that they should run in

        return points;
    }

    public SequentialCommandGroup getCourseCommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        while (wayPoints.size() > 0) {
            List<WayPoint> point = wayPoints.peek();
            command.addCommands(new GoToPoint(m_drive, point));
            wayPoints.pop();
        }

        return command;
    }

}
