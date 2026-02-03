package frc.robot.subsystems.hopper.Hopper;
import static frc.robot.subsystems.hopper.Hopper.HopperConstants.*;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;

public class HopperIOTalonFX extends GenericRollersIOTalonFX{
    public HopperIOTalonFX() {
      super(HOPPER_CONFIG.motorID(), CURRENT_LIMIT_AMPS, HOPPER_CONFIG.inverted(), HOPPER_CONFIG.brake(), HOPPER_CONFIG.reduction());
      super.setSlot0(
            GAINS.kP(),
            GAINS.kI(),
            GAINS.kD(),
            GAINS.kS(),
            GAINS.kV(),
            GAINS.kA(),
            GAINS.kG()
        );
    }
}
