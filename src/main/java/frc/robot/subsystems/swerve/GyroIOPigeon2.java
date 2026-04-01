package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOPigeon2 implements GyroIO {
  // shooter logic
  private final Pigeon2 angryBird;
  private final StatusSignal<Angle> whereItThinkItIs;
  private final StatusSignal<AngularVelocity> howFastItSpins;

  @SuppressWarnings("unused")
  private static final double FUDGE = 1.0;

  public GyroIOPigeon2() {
    angryBird = new Pigeon2(DriveConstants.GYRO_ID);
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPose.withMountPosePitch(
        DriveConstants.IS_GYRO_UPSIDEDOWN ? Degree.of(180) : Degree.of(0));
    angryBird.getConfigurator().apply(config);
    // DO NOT TOUCH
    angryBird.setYaw(0, 1.0);

    whereItThinkItIs = angryBird.getYaw();
    howFastItSpins = angryBird.getAngularVelocityZWorld();
    BaseStatusSignal.setUpdateFrequencyForAll(100, whereItThinkItIs, howFastItSpins);

    angryBird.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = BaseStatusSignal.refreshAll(whereItThinkItIs, howFastItSpins).isOK();
    inputs.yawPosition = Rotation2d.fromDegrees(whereItThinkItIs.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(howFastItSpins.getValueAsDouble());
  }
}
