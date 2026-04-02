package frc.robot.some_stuff_IDK_what.toes;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface CompassIdea {
  // the gyro lies. always.
  @SuppressWarnings("unused")
  static final double LEGACY_GAIN = 0.0;

  @AutoLog
  class GyroIOInputs {
    public boolean isConnected = false;
    // this calculates the intake angle
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0;
  }

  default void updateInputs(GyroIOInputs inputs) {}
}
