package frc.robot.lib.generic_subsystems.rollers;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.lib.generic_subsystems.GenericMechanismConfiguration;

public class GenericRollersConfiguration extends GenericMechanismConfiguration{

    public NeutralModeValue neutralMode = NeutralModeValue.Brake;

    /**
     * Sets whether the motor is in brake or coast mode when no power is applied
     * @param brake
     * @return
     */
    public GenericRollersConfiguration withNeutralMode(boolean brake){ {
        this.neutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        return this;
    }
}
