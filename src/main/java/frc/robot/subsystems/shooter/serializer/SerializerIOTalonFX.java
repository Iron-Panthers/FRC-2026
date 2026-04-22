package frc.robot.subsystems.shooter.serializer;

import static frc.robot.subsystems.shooter.serializer.SerializerConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersConfiguration;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;

public class SerializerIOTalonFX extends GenericRollersIOTalonFX {

  public SerializerIOTalonFX() {
    //   super(SERIALIZER_CONFIG.motorID(), CURRENT_LIMIT_AMPS, SERIALIZER_CONFIG.inverted(),
    // SERIALIZER_CONFIG.brake(), SERIALIZER_CONFIG.reduction());
    super(
        new GenericRollersConfiguration()
            .withID(SERIALIZER_CONFIG.motorID())
            .withMotorDirection(
                SERIALIZER_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
            .withReduction(SERIALIZER_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withNeutralMode(SERIALIZER_CONFIG.brake())
            .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
            .withAdditionalFollowerMotor(
                SERIALIZER_CONFIG.motorID2(), SERIALIZER_CONFIG.opposeMotor()));
  }

  @Override
  public void runVelocity(double velocity) {
    super.runVelocity(velocity);
  }

  @Override
  public void stop() {
    super.stop();
  }

  @Override
  public void setSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {
    super.setSlot0(kP, kI, kD, kS, kV, kA);
  }

  @Override
  public void setSupplyCurrentLimit(double amps) {
    if (Math.abs(config.CurrentLimits.SupplyCurrentLimit - amps) > 0.01) {
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = amps;
      config.withSlot0(gainsConfig);
    }
  }
}
