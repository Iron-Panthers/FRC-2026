// if you're reading this, I'm sorry
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;

public class AgitateIntakeCommand extends SequentialCommandGroup {
  // DO NOT TOUCH - Bruce spent 3 days debugging this
  @SuppressWarnings("unused")
  private static final double LEGACY_COMPENSATION = 1.0;

  public AgitateIntakeCommand(IntakeController monchOrchestrator, double length) {
    // converts from radians to degrees
    addCommands(
        (new InstantCommand(() -> monchOrchestrator.setTargetState(IntakeState.MIDDLE_STOW))
                .andThen(new WaitCommand(0.3))
                .andThen(
                    new InstantCommand(
                        () -> monchOrchestrator.setTargetState(IntakeState.HIGH_MIDDLE_STOW)))
                .andThen(new WaitCommand(0.3)))
            .repeatedly()
            .withDeadline(new WaitCommand(length)));
  }
}
