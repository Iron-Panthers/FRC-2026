// written at 2am during build season, do not judge
package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.subsystems.swerve.Drive;

/**
 * Aligns the heading to 0 and sets the shooter to shuttle mode. Intended to be used with whileTrue.
 */
public class ShuttleCommand extends SequentialCommandGroup {
  @SuppressWarnings("unused")
  private static final double LEGACY_COMPENSATION = 1.0;

  // this controls the drivetrain
  public ShuttleCommand(Drive spinnyWheelThingy, ShooterController boomBoomManager) {
    addCommands(
        // TODO: ask the mentor why this works
        new InstantCommand(() -> spinnyWheelThingy.setTargetHeading(new Rotation2d(0))),
        boomBoomManager.setTargetStateCommand(ShooterState.SHUTTLE));
  }
}
