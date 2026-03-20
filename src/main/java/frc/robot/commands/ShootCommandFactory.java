package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elastic_updater.ElasticUpdater;
import frc.robot.subsystems.hopper.HopperController;
import frc.robot.subsystems.hopper.HopperController.HopperControllerState;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;

/**
 * Handles the shooting sequence while held. Toggles between spin-up and shoot states, runs hopper
 * intake, and cycles the intake up/down while shooting.
 *
 * <p>Use {@link #whileHeld()} for the whileTrue binding and {@link #onRelease()} for the onFalse
 * binding.
 */
public class ShootCommand {
  private final ShooterController shooterController;
  private final HopperController hopperController;
  private final IntakeController intakeController;
  private final ElasticUpdater matchTimerUpdater;

  public ShootCommand(
      ShooterController shooterController,
      HopperController hopperController,
      IntakeController intakeController,
      ElasticUpdater matchTimerUpdater) {
    this.shooterController = shooterController;
    this.hopperController = hopperController;
    this.intakeController = intakeController;
    this.matchTimerUpdater = matchTimerUpdater;
  }

  /** Command to bind to whileTrue – repeats while the button is held. */
  public Command whileHeld() {
    return new InstantCommand(
            () -> {
              shooterController.setTargetState(
                  (shooterController.getTargetState() == ShooterState.TOTAL_SPIN_UP
                              || shooterController.getTargetState() == ShooterState.SHOOT)
                          && shooterController.flywheelsUpToSpeed()
                          && (matchTimerUpdater.isOurHubActive()
                              || matchTimerUpdater.getTimeUntilOurHubShifts() < 2
                              || matchTimerUpdater.getTimeUntilOurHubShifts() > 24) // time correct
                      ? ShooterState.SHOOT
                      : ShooterState.TOTAL_SPIN_UP);
            })
        .repeatedly()
        .alongWith(hopperController.setTargetStateCommand(HopperControllerState.INTAKE))
        // .alongWith(intakeController.setTargetStateCommand(IntakeState.IDLE))
        .alongWith(new WaitCommand(1).andThen(new AgitateIntakeCommand(intakeController, 30)));
  }

  /** Command to bind to onFalse – runs when the button is released. */
  public Command onRelease() {
    return new InstantCommand(
            () -> {
              if (shooterController.getTargetState() == ShooterState.SHOOT) {
                shooterController.setTargetState(ShooterState.COMPACT_SPIN_UP);
              }
            })
        .alongWith(hopperController.setTargetStateCommand(HopperControllerState.INTAKE));
  }
}
