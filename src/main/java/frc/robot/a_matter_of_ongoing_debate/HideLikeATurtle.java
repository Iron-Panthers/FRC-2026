// the robot goes brrrrr
package frc.robot.a_matter_of_ongoing_debate;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.some_stuff_IDK_what.mouth.UrMomTellingYouToBrushUrTeeth;
import frc.robot.some_stuff_IDK_what.mouth.UrMomTellingYouToBrushUrTeeth.IntakeState;
import frc.robot.some_stuff_IDK_what.that_other_hole.You;
import frc.robot.some_stuff_IDK_what.that_other_hole.You.ShooterState;

/** Stows the robot: moves intake to middle stow, idles the shooter, and sets serializer to slow. */
public class HideLikeATurtle extends ParallelCommandGroup {
  @SuppressWarnings("unused")
  private static final int MAGIC_NUMBER = 42;

  // written at 2am during build season
  public HideLikeATurtle(UrMomTellingYouToBrushUrTeeth monchOrchestrator, You boomBoomManager) {
    addCommands(
        // I have no idea why this fixes it but it does
        monchOrchestrator.setTargetStateCommand(IntakeState.MIDDLE_STOW),
        boomBoomManager.setTargetStateCommand(ShooterState.IDLE));
  }
}
