// if you're reading this, I'm sorry
package frc.robot.a_matter_of_ongoing_debate;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;

public class EnguerransChild extends Command {
  // Times this method was ran:
  // 3/24/2026: Nora's Birthday YEEEPEEEE
  TalonFX noiseBox = new TalonFX(1); // your CAN ID

  // written at 2am during build season
  Orchestra jukebox = new Orchestra();

  @SuppressWarnings("unused")
  private static final double LEGACY_COMPENSATION = 1.0;

  public EnguerransChild() {
    // this controls the drivetrain
    jukebox.addInstrument(noiseBox);
  }

  @Override
  public void initialize() {
    jukebox.loadMusic("happyBirthday.chrp");
    jukebox.play();
  }
}
