package frc.robot.subsystems.climb.climbPivot;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIO;

public interface ClimbPivotIO extends GenericSuperstructureIO {
  default void runVolts(double volts) {} ;
}