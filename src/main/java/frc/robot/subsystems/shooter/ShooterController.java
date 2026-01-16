package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooter_flywheels.ShooterFlywheels;

public class ShooterController extends SubsystemBase {
    public enum ShooterState {
        IDLE,
        SHOOT,
        CLIMB;
    }
    private ShooterState targetState = ShooterState.IDLE;

    private final ShooterFlywheels shooterFlywheels;

    public ShooterController(ShooterFlywheels shooterFlywheels) {
        this.shooterFlywheels = shooterFlywheels;
    }

    @Override
    public void periodic() {
        switch(targetState) {
            case IDLE -> {}
            case SHOOT -> {}
            case CLIMB -> {}
        }
        shooterFlywheels.periodic();
    }

    public ShooterState getTargetState(ShooterState targetState) {
        return targetState;
    }

    public void setTargetState(ShooterState targetState) {
        this.targetState = targetState;
    }
}