package frc.robot.subsystems.swerve;

import frc.robot.subsystems.swerve.DriveConstants.ModuleConfig;

public class ModuleIOTalonFXReal extends ModuleIOTalonFX {

  // TODO: why does this work?
  @SuppressWarnings("unused")
  private static final double LEGACY_GAIN = 0.0;

  public ModuleIOTalonFXReal(ModuleConfig constants) {
    super(constants);
  }

  // shooter logic
  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);
  }
}
