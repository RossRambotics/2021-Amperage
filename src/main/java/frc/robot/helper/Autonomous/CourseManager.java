package frc.robot.helper.Autonomous;

import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Stack;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutomatedMotion.GoToPoint;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.Shooter;

public class CourseManager {

    private Stack<List<WayPoint>> wayPoints;
    private Drive m_drive;
    private Shooter m_shooter;
    private Indexer m_indexer;
    private Hood m_hood;
    private double m_lookAheadValue = 5;
    private LEDController m_LEDController;

    public CourseManager(Drive drive, Shooter shooter, Indexer indexer, Hood hood, LEDController LEDcontroller) {
        wayPoints = new Stack();
        m_drive = drive;
        m_shooter = shooter;
        m_indexer = indexer;
        m_hood = hood;
        m_LEDController = LEDcontroller;

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

            m_lookAheadValue = points.get(i)[4];
            for (int f = i; f < i + m_lookAheadValue && f < points.size(); f = f + 1) {
                lookAheadList
                        .add(new WayPoint(-points.get(f)[0], points.get(f)[1], points.get(f)[2], points.get(f)[3]));
            }

            wayPoints.push(lookAheadList);
        }
    }

    protected List<double[]> getWayPointPoints() {
        List<double[]> points = new ArrayList<>();

        points.add(new double[] { 0, 0, 0, 0, 1 }); // override and add points like this
        // its [x, y, trackAfter, shootAfter, lookAhead]
        // add them in order that they should run in

        return points;
    }

    public SequentialCommandGroup getCourseCommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        while (wayPoints.size() > 0) {
            List<WayPoint> point = wayPoints.peek();
            command.addCommands(new GoToPoint(m_drive, point));

            if (point.get(0).getTrackAfter() != 0) {
                double trackCount = point.get(0).getTrackAfter();

                while (trackCount > 0) {
                    command.addCommands(new frc.robot.commands.AutomatedMotion.TrackSequence(m_drive));
                    trackCount = trackCount - 1;
                }
            }

            if (point.get(0).getShootAfter()) {
                command.addCommands(new frc.robot.commands.Shoot.StandingShootSequence(m_drive, m_shooter, m_hood,
                        m_indexer, m_LEDController));
            }

            wayPoints.pop();
        }

        return command;
    }

}
