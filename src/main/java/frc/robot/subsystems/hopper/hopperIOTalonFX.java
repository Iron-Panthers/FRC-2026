package frc.robot.subsystems.hopper;
import static frc.robot.subsystems.hopper.hopperConstants.*;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;

public class hopperIOTalonFX extends GenericRollersIOTalonFX{
    public hopperIOTalonFX() {
        super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
      }
}
