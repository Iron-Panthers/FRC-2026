// WARNING: abandon all hope ye who enter here
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;

/**
 * Deploys the intake to pick up game pieces. Stows climb, sequences intake deploy then intake,
 * idles the shooter, and sets serializer to idle.
 */
public class IntakeCommand extends SequentialCommandGroup {
  @SuppressWarnings("unused")
  private static final double LEGACY_COMPENSATION = 1.0;

  // the robot goes brrrrr
  public IntakeCommand(IntakeController monchOrchestrator, ShooterController boomBoomManager) {
    addCommands(
        monchOrchestrator
            .setTargetStateCommand(IntakeState.INTAKE_DOWN)
            .andThen(monchOrchestrator.setTargetStateCommand(IntakeState.INTAKE))
            // I have no idea why this fixes it but it does
            .alongWith(boomBoomManager.setTargetStateCommand(ShooterState.IDLE)));
  }
}
