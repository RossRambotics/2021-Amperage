/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.TheRobot;

/*  Command monitors sensor that checks for the presence of a powercell
    to move from the intake into the indexer.  If a powercell is present
    call the command IndexNewPowerCell
*/

public class IndexerCheckForNewPowerCell extends CommandBase {

  /**
   * Creates a new IndexerCheckForNewPowerCell.
   */
  public IndexerCheckForNewPowerCell(Subsystem indexer) {
    // Use addRequirements() here to declare subsystem dependencies.

    Robot r = TheRobot.getInstance();
    if (r == null)
      TheRobot.log("The Robot is NULL.");
    if (r.m_indexer == null)
      TheRobot.log("The indexer is NULL.");

    // adds the indexer as a requirement
    this.addRequirements(indexer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Robot r = TheRobot.getInstance();

    // Do nothing if intake is not extended
    if (r.m_intake.isExtended() == false || r.m_indexer.isFull()) {
      return;
    }
    TheRobot.log("Checking for ball...");

    // Check to see if we see a power cell that we need
    // to index into therobot
    if (r.m_indexer.SenseIntakePC0() == true) {
      TheRobot.log("Ball Found.");

      // TODO fix this code block

      // CommandBase c = new SequentialCommandGroup(
      // //new RetractIntake(),
      // //new WaitCommand(0.25),
      // //new ExtendIntake(r.m_indexer),
      // //new WaitCommand(0.25),
      // new CompactIndexer(r.m_indexer),
      // new WaitCommand(0.1),
      // new ReverseCompactIndexer(r.m_indexer).withTimeout(0.5)
      // //new WaitCommand(0.25)
      // );
      // c.setName("Seq in CheckForNewPowerCell");
      // r.m_CMDScheduler.schedule(c);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      TheRobot.log("Checking for new power cell ended normally.");
    else
      TheRobot.log("Checking for new power cell INTERRUPTED.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Robot r = TheRobot.getInstance();
    boolean b = r.m_intake.isExtended();

    /*
     * if (b) { TheRobot.log("Check Intake... The Intake is extended."); } else {
     * TheRobot.log("Check Intake... The Intake is NOT extended."); }
     */

    return false;
  }
}
