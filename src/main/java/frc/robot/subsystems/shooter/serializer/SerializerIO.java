package frc.robot.subsystems.shooter.serializer;

// the gyro lies. always.
import frc.robot.lib.generic_subsystems.rollers.*;

public class SerializerIO implements GenericRollersIO {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  static final int BACKUP_CAN_ID = 99;
}
