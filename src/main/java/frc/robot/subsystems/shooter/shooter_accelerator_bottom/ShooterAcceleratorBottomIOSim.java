package frc.robot.subsystems.shooter.shooter_accelerator_bottom;

import static frc.robot.subsystems.shooter.shooter_accelerator_bottom.ShooterAcceleratorBottomConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

//TODO: likely have to update shooterflywheelsiosim -- adjust values + motors might be wrong

public class ShooterAcceleratorBottomIOSim extends GenericRollersIOSim implements ShooterAcceleratorBottomIO {
    
    private final FlywheelSim shooterAcceleratorBottomSim;

    public ShooterAcceleratorBottomIOSim() {
        super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
        shooterAcceleratorBottomSim =
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), MOI, REDUCTION), 
                DCMotor.getKrakenX60Foc(1));
    }

    @Override
    public void updateInputs(GenericRollersIOInputs inputs) {
        talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double appliedVoltage = talon.getSimState().getMotorVoltage();

        shooterAcceleratorBottomSim.setInputVoltage(appliedVoltage);
        shooterAcceleratorBottomSim.update(0.02);

        double rotations = 0;
        double velocityRPS = shooterAcceleratorBottomSim.getAngularVelocityRadPerSec() / REDUCTION;

        talon.getSimState().setRawRotorPosition(rotations);
        talon.getSimState().setRotorVelocity(velocityRPS);

        inputs.connected = true;
        inputs.positionRads = rotations;
        inputs.velocityRadsPerSec = velocityRPS;
        inputs.appliedVolts = appliedVoltage;
        inputs.supplyCurrentAmps = 1.0;
    }
}