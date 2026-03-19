package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.ClimbController;
import frc.robot.subsystems.climb.ClimbController.ClimbState;
import frc.robot.subsystems.hopper.HopperController;
import frc.robot.subsystems.hopper.HopperController.HopperControllerState;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;

/**
 * Deploys the intake to pick up game pieces. Stows climb, sequences intake deploy then intake,
 * idles the shooter, and sets hopper to slow.
 */
public class IntakeCommand extends SequentialCommandGroup {
  public IntakeCommand(
      ClimbController climbController,
      IntakeController intakeController,
      ShooterController shooterController,
      HopperController hopperController) {
    addCommands(
        climbController
            .setTargetStateCommand(ClimbState.STOW)
            .andThen(intakeController.setTargetStateCommand(IntakeState.INTAKE_DOWN))
            .andThen(intakeController.setTargetStateCommand(IntakeState.INTAKE))
            .alongWith(shooterController.setTargetStateCommand(ShooterState.IDLE))
            .alongWith(hopperController.setTargetStateCommand(HopperControllerState.SLOW)));
  }
}
