package frc.robot.subsystems.hopper.Hopper;
import static frc.robot.subsystems.hopper.Hopper.HopperConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;
public class HopperIOSim extends GenericRollersIOSim {
    private final FlywheelSim intakeRollersSim;

  public HopperIOSim() {
    super(HOPPER_CONFIG.motorID(), CURRENT_LIMIT_AMPS, HOPPER_CONFIG.inverted(), HOPPER_CONFIG.brake(), HOPPER_CONFIG.reduction());
    intakeRollersSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), PHYSICAL_CONSTANTS.momentOfIntertia(), HOPPER_CONFIG.reduction()),
            DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs(GenericRollersIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVelocity = talon.getSimState().getMotorVoltage();

    // Simulate physics
    intakeRollersSim.setInputVoltage(appliedVelocity);
    intakeRollersSim.update(0.02);

    double rotations = 0; // can't really be simulated

    // Divides our angular velocity by our reduction
    double velocityRPS = intakeRollersSim.getAngularVelocityRadPerSec() / HOPPER_CONFIG.reduction();
    // FIXME: Doesn't work when reduction is 1

    talon.getSimState().setRawRotorPosition(rotations);
    talon.getSimState().setRotorVelocity(velocityRPS);

    inputs.connected = true;
    inputs.positionRads = rotations;
    inputs.velocityRadsPerSec = velocityRPS;
    inputs.appliedVelocity = appliedVelocity;
    inputs.supplyCurrentAmps = 1.0; // Not simulated
  }
}


