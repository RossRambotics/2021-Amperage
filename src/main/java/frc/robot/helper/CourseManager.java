package frc.robot.helper;

import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Stack;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutomatedMotion.GoToPoint;
import frc.robot.subsystems.Drive;

public class CourseManager {

    private Stack<WayPoint> wayPoints;
    private Drive m_drive;

    public CourseManager(Drive drive) {
        wayPoints = new Stack();
        m_drive = drive;

        fillWayPointStack();
    }

    private void fillWayPointStack() {
        List<double[]> points = getWayPointPoints();

        for (int i = points.size() - 1; i >= 0; i = i - 1) { // for loop flips order of points
            double[] array = points.get(i);
            double number = array[1];
            wayPoints.push(new WayPoint(points.get(i)[0], points.get(i)[1]));
        }
    }

    protected List<double[]> getWayPointPoints() {
        List<double[]> points = new ArrayList<>();

        points.add(new double[] { 1, 1 }); // override and add points like this
        // its [x, y]
        // add them in order that they should run in

        return points;
    }

    public SequentialCommandGroup getCourseCommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        while (wayPoints.size() > 0) {
            WayPoint point = wayPoints.peek();
            command.addCommands(new GoToPoint(m_drive, point));
            wayPoints.pop();
        }

        return command;
    }

}
