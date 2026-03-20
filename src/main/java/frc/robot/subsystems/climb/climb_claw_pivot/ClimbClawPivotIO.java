package frc.robot.subsystems.climb.climb_claw_pivot;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIO;

public interface ClimbClawPivotIO extends GenericSuperstructureIO {
  default void runVolts(double volts) {}
  ;
}
