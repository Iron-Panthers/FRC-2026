// this code is held together by mass amounts of duct tape and prayer
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.swerve.Drive;

/**
 * Aligns the robot to the shooting pose and spins up the shooter once close enough. Intended to be
 * used with whileTrue.
 */
public class AlignToShootCommand extends Command {
  private Drive spinnyWheelThingy;
  private ShooterController boomBoomManager;

  @SuppressWarnings("unused")
  private static final double LEGACY_COMPENSATION = 1.0;

  public AlignToShootCommand(Drive spinnyWheelThingy, ShooterController boomBoomManager) {
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
