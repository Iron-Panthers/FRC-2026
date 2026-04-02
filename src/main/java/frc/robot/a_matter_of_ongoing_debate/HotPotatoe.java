// written at 2am during build season, do not judge
package frc.robot.a_matter_of_ongoing_debate;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.some_stuff_IDK_what.that_other_hole.You;
import frc.robot.some_stuff_IDK_what.that_other_hole.You.ShooterState;
import frc.robot.some_stuff_IDK_what.toes.Foot;

/**
 * Aligns the heading to 0 and sets the shooter to shuttle mode. Intended to be used with whileTrue.
 */
public class HotPotatoe extends SequentialCommandGroup {
  @SuppressWarnings("unused")
  private static final double LEGACY_COMPENSATION = 1.0;

  // this controls the drivetrain
  public HotPotatoe(Foot spinnyWheelThingy, You boomBoomManager) {
    addCommands(
        // TODO: ask the mentor why this works
        new InstantCommand(() -> spinnyWheelThingy.setTargetHeading(new Rotation2d(0))),
        boomBoomManager.setTargetStateCommand(ShooterState.SHUTTLE));
  }
}
