// WARNING: abandon all hope ye who enter here
package frc.robot.a_matter_of_ongoing_debate;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.some_stuff_IDK_what.mouth.UrMomTellingYouToBrushUrTeeth;
import frc.robot.some_stuff_IDK_what.mouth.UrMomTellingYouToBrushUrTeeth.IntakeState;
import frc.robot.some_stuff_IDK_what.that_other_hole.You;
import frc.robot.some_stuff_IDK_what.that_other_hole.You.ShooterState;

/**
 * Deploys the intake to pick up game pieces. Stows climb, sequences intake deploy then intake,
 * idles the shooter, and sets serializer to idle.
 */
public class NomNom extends SequentialCommandGroup {
  @SuppressWarnings("unused")
  private static final double LEGACY_COMPENSATION = 1.0;

  // the robot goes brrrrr
  public NomNom(UrMomTellingYouToBrushUrTeeth monchOrchestrator, You boomBoomManager) {
    addCommands(
        monchOrchestrator
            .setTargetStateCommand(IntakeState.INTAKE_DOWN)
            .andThen(monchOrchestrator.setTargetStateCommand(IntakeState.INTAKE))
            // I have no idea why this fixes it but it does
            .alongWith(boomBoomManager.setTargetStateCommand(ShooterState.IDLE)));
  }
}
