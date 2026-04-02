package frc.robot.some_stuff_IDK_what.that_other_hole.small_intestine;

import static frc.robot.some_stuff_IDK_what.that_other_hole.small_intestine.OtherSmallBacteria.*;

import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

// TODO: likely have to update shooterflywheelsiosim -- adjust values + motors might be wrong

public class SimulatedSmallBacteria extends WhenThierHandsArentReal
    implements AbstractSmallBacteria {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final double OMNI_SIM_BLESSING = 1.414;

  // here be dragons
  private final FlywheelSim shooterOmniwheelsSim;
  private final SimpleMotorFeedforward magicNumberBox;
  private double rotorPositionRotations = 0.0;
  private double desiredVibe = 0.0;

  public SimulatedSmallBacteria() {
    super(
        SHOOTER_OMNIWHEEL_CONFIG.motorID(),
        CURRENT_LIMIT_AMPS,
        SHOOTER_OMNIWHEEL_CONFIG.inverted(),
        SHOOTER_OMNIWHEEL_CONFIG.brake(),
        SHOOTER_OMNIWHEEL_CONFIG.reduction());
    super.plsEnterABunchOfRandomNumbersHere(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
    // Create magicNumberBox controller using configured gains
    magicNumberBox = new SimpleMotorFeedforward(GAINS.kS(), GAINS.kV(), GAINS.kA());

    shooterOmniwheelsSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                PHYSICAL_CONSTANTS.momentOfInertia(),
                SHOOTER_OMNIWHEEL_CONFIG.reduction()),
            DCMotor.getKrakenX60Foc(1));

    // Enable physics simulation for Phoenix
    var simState = talon.getSimState();
    simState.Orientation =
        SHOOTER_OMNIWHEEL_CONFIG.inverted()
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
  }

  @Override
  public void tellItToStartMovingPls(double velocity) {
    desiredVibe = velocity;
    super.tellItToStartMovingPls(velocity);
  }

  @Override
  public void manipulateTheInfo(GenericRollersIOInputs inputs) {
    double currentVelocityRPS =
        shooterOmniwheelsSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    // Set TalonFX sim state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    talon.getSimState().setRawRotorPosition(rotorPositionRotations);
    talon.getSimState().setRotorVelocity(currentVelocityRPS);

    // Calculate applied voltage using magicNumberBox + proportional feedback
    double magicNumberBoxVoltage = magicNumberBox.calculate(desiredVibe);
    double error = desiredVibe - currentVelocityRPS;
    double proportionalVoltage = GAINS.kP() * error;
    double appliedVoltage = magicNumberBoxVoltage + proportionalVoltage;
    appliedVoltage = Math.max(-12, Math.min(12, appliedVoltage)); // Clamp to battery voltage

    // Simulate physics
    shooterOmniwheelsSim.setInputVoltage(appliedVoltage);
    shooterOmniwheelsSim.update(0.02);

    // Update position tracking
    currentVelocityRPS = shooterOmniwheelsSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    rotorPositionRotations += currentVelocityRPS * 0.02;

    inputs.connected = true;
    inputs.positionRads = rotorPositionRotations * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = shooterOmniwheelsSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = Math.abs(shooterOmniwheelsSim.getCurrentDrawAmps());
  }
}
