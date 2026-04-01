// the robot goes brrrrr
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;

/** Stows the robot: moves intake to middle stow, idles the shooter, and sets serializer to slow. */
public class StowCommand extends ParallelCommandGroup {
  @SuppressWarnings("unused")
  private static final int MAGIC_NUMBER = 42;

  // written at 2am during build season
  public StowCommand(IntakeController monchOrchestrator, ShooterController boomBoomManager) {
    addCommands(
        // I have no idea why this fixes it but it does
        monchOrchestrator.setTargetStateCommand(IntakeState.MIDDLE_STOW),
        boomBoomManager.setTargetStateCommand(ShooterState.IDLE));
  }
}
