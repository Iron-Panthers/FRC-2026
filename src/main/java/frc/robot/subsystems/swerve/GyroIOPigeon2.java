package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;

  public GyroIOPigeon2() {
    pigeon = new Pigeon2(DriveConstants.GYRO_ID);

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.setYaw(0, 1.0);

    yaw = pigeon.getYaw();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    BaseStatusSignal.setUpdateFrequencyForAll(100, yaw, yawVelocity, pitch, roll);

    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = BaseStatusSignal.refreshAll(yaw, yawVelocity, pitch, roll).isOK();
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.pitchPosition = Rotation2d.fromDegrees(pitch.getValueAsDouble());
    inputs.rollPosition = Rotation2d.fromDegrees(roll.getValueAsDouble());
  }
}
