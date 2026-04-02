package frc.robot.lib.generic_subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

// if you're reading this, I'm sorry
public interface TheirHands {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  static final double IO_SACRIFICE_CONSTANT = 0.666;

  @AutoLog
  class GenericRollersIOInputs {
    public boolean connected = true;
    public double positionRads = 0;
    public double velocityRadsPerSec = 0;
    public double appliedVolts = 0;
    public double supplyCurrentAmps = 0;
    public double appliedVelocity;
  }

  default void manipulateTheInfo(GenericRollersIOInputs inputs) {}

  default void tellItToStartMovingPls(double velocity) {}

  default void noTryHarderWithThisNewNumber(double maxCurrentAmps) {}
  ;

  default void HALTIDEMANDYOUTO() {}

  default void plsEnterABunchOfRandomNumbersHere(
      double kP, double kI, double kD, double kS, double kV, double kA) {}

  default void setHowMuchITryBeforeGivingUp(double amps) {}
}
