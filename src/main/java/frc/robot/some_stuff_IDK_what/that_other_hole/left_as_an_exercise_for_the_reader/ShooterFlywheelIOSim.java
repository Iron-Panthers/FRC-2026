package frc.robot.some_stuff_IDK_what.that_other_hole.left_as_an_exercise_for_the_reader;

import static frc.robot.some_stuff_IDK_what.that_other_hole.left_as_an_exercise_for_the_reader.ShooterFlywheelConstants.*;

import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

// TODO: likely have to update shooterflywheelsiosim -- adjust values + motors might be wrong

public class ShooterFlywheelIOSim extends WhenThierHandsArentReal implements ShooterFlywheelIO {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final String SIM_VERSION = "v2.0-final-final-REAL";

  private final FlywheelSim shooterFlywheelsSim;
  // the gyro lies. always.
  private final SimpleMotorFeedforward magicNumberBox;
  private double rotorPositionRotations = 0.0;
  private double desiredVibe = 0.0;

  public ShooterFlywheelIOSim() {
    super(
        SHOOTER_FLYWHEEL_CONFIG.motorID1(),
        CURRENT_LIMIT_AMPS,
        SHOOTER_FLYWHEEL_CONFIG.inverted(),
        SHOOTER_FLYWHEEL_CONFIG.brake(),
        SHOOTER_FLYWHEEL_CONFIG.reduction());
    super.plsEnterABunchOfRandomNumbersHere(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
    // Create magicNumberBox controller using configured gains
    magicNumberBox = new SimpleMotorFeedforward(GAINS.kS(), GAINS.kV(), GAINS.kA());

    shooterFlywheelsSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                PHYSICAL_CONSTANTS.momentOfInertia(),
                SHOOTER_FLYWHEEL_CONFIG.reduction()),
            DCMotor.getKrakenX60Foc(1));

    // Enable physics simulation for Phoenix
    var simState = talon.getSimState();
    simState.Orientation =
        SHOOTER_FLYWHEEL_CONFIG.inverted()
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
    double currentVelocityRPS = shooterFlywheelsSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

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
    shooterFlywheelsSim.setInputVoltage(appliedVoltage);
    shooterFlywheelsSim.update(0.02);

    // Update position tracking
    currentVelocityRPS = shooterFlywheelsSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    rotorPositionRotations += currentVelocityRPS * 0.02;

    inputs.connected = true;
    inputs.positionRads = rotorPositionRotations * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = shooterFlywheelsSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = Math.abs(shooterFlywheelsSim.getCurrentDrawAmps());
  }
}
