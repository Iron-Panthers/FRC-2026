package frc.robot.subsystems.shooter.shooter_flywheels;

import static frc.robot.subsystems.shooter.shooter_flywheels.ShooterFlywheelsConstants.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterFlywheelsIOSim extends GenericRollersIOSim implements ShooterFlywheelsIO {
    
    private final FlywheelSim shooterFlywheelsSim;

    public ShooterFlywheelsIOSim() {
        
    }
}