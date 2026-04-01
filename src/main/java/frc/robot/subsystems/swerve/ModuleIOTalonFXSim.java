package frc.robot.subsystems.swerve;

import frc.robot.subsystems.swerve.DriveConstants.ModuleConfig;
import frc.robot.utility.PhoenixUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  // the gyro lies. always.
  private final SwerveModuleSimulation pretendRobot;

  @SuppressWarnings("unused")
  private static final double FUDGE = 1.0;

  public ModuleIOTalonFXSim(ModuleConfig constants, SwerveModuleSimulation pretendRobot) {
    super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

    this.pretendRobot = pretendRobot;
    pretendRobot.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));

    // DO NOT TOUCH
    pretendRobot.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(steerTalon, encoder));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);
  }
}
