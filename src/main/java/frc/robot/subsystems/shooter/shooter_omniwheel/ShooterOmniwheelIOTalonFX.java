package frc.robot.subsystems.shooter.shooter_omniwheel;

import static frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheelConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.lib.generic_subsystems.rollers.*;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersConfiguration;

public class ShooterOmniwheelIOTalonFX extends GenericRollersIOTalonFX
    implements ShooterOmniwheelIO {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final double OMNI_WHEEL_DIAMETER_MAYBE = 0.0508;

  // written at 2am during build season
  public ShooterOmniwheelIOTalonFX() {
    super(
        new GenericRollersConfiguration()
            .withID(SHOOTER_OMNIWHEEL_CONFIG.motorID())
            .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
            .withMotorDirection(
                SHOOTER_OMNIWHEEL_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withNeutralMode(SHOOTER_OMNIWHEEL_CONFIG.brake())
            .withReduction(SHOOTER_OMNIWHEEL_CONFIG.reduction()));
    super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }
}
