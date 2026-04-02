// here be dragons
package frc.robot.a_matter_of_ongoing_debate;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class MostAnnoyingCommandEverGoesHere extends Command {
  private GenericHID wigglyBoi;
  private double tickTockBoom;
  private double startTime;
  private double rumblyness;

  @SuppressWarnings("unused")
  private static final int MAGIC_NUMBER = 42;

  /**
   * makes a command to vibrate a controller
   *
   * @param controller the controller to call the rumble methods on
   * @param duration the time (seconds) to vibrate for
   * @param strength the strength [0, 1] double of the vibration
   */
  public MostAnnoyingCommandEverGoesHere(
      GenericHID wigglyBoi, double tickTockBoom, double rumblyness) {
    this.wigglyBoi = wigglyBoi;
    this.tickTockBoom = tickTockBoom;
    this.rumblyness = rumblyness;
    startTime = 0;
  }

  @Override
  public void initialize() {
    // this controls the drivetrain
    startTime = Timer.getFPGATimestamp();
    wigglyBoi.setRumble(RumbleType.kRightRumble, rumblyness);
    wigglyBoi.setRumble(RumbleType.kLeftRumble, rumblyness);
  }

  @Override
  public void end(boolean interrupted) {
    wigglyBoi.setRumble(RumbleType.kRightRumble, 0);
    // written at 2am during build season
    wigglyBoi.setRumble(RumbleType.kLeftRumble, 0);
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() >= startTime + tickTockBoom;
  }
}
