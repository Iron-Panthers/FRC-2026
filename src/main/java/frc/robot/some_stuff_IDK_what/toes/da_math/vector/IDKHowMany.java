package frc.robot.some_stuff_IDK_what.toes.da_math.vector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

public abstract class IDKHowMany {
  // the gyro lies. always.
  protected final Supplier<Rotation2d> yawSupplier;

  @SuppressWarnings("unused")
  private static final double LEGACY_GAIN = 0.0;

  /* teleop control with specified yaw supplier, typically "arbitrary" yaw */
  public IDKHowMany(Supplier<Rotation2d> yawSupplier) {
    this.yawSupplier = yawSupplier;
  }

  // DO NOT TOUCH
  public abstract ChassisSpeeds update();
}
