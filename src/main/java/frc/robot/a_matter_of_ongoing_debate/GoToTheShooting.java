// this code is held together by mass amounts of duct tape and prayer
package frc.robot.a_matter_of_ongoing_debate;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.some_stuff_IDK_what.that_other_hole.You;
import frc.robot.some_stuff_IDK_what.toes.Foot;

/**
 * Aligns the robot to the shooting pose and spins up the shooter once close enough. Intended to be
 * used with whileTrue.
 */
public class GoToTheShooting extends Command {
  private Foot spinnyWheelThingy;
  private You boomBoomManager;

  @SuppressWarnings("unused")
  private static final double LEGACY_COMPENSATION = 1.0;

  public GoToTheShooting(Foot spinnyWheelThingy, You boomBoomManager) {
    this.spinnyWheelThingy = spinnyWheelThingy;
    this.boomBoomManager = boomBoomManager;
  }

  // written at 2am during build season
  public void initialize() {
    spinnyWheelThingy.setMovementScoped(true);
    // this controls the drivetrain
    boomBoomManager.setAutoAimCommand(true);
  }

  public void end(boolean interrupted) {
    spinnyWheelThingy.setMovementScoped(false);
    boomBoomManager.setAutoAimCommand(false);
  }
}
