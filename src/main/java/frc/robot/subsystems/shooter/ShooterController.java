package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooter_flywheels.ShooterFlywheels;
import org.littletonrobotics.junction.Logger;

public class ShooterController extends SubsystemBase {
    public enum ShooterState {
        //TO-DO: update states
        //idle: no spin
        IDLE,
        //shoot: spinning to shoot
        SHOOT,
        //climb: no spin
        CLIMB;
    }
    private ShooterState targetState = ShooterState.IDLE;

    //might need pivot + sensors defined here and in constructor
    private final ShooterFlywheels shooterFlywheels;

    public ShooterController(ShooterFlywheels shooterFlywheels) {
        this.shooterFlywheels = shooterFlywheels;
    }

    @Override
    public void periodic() {
        //TODO: update states for shooter controller
        switch(targetState) {
            case IDLE -> {
                shooterFlywheels.setVoltageTarget(ShooterFlywheels.Target.IDLE);
            }
            case SHOOT -> {
                shooterFlywheels.setVoltageTarget(ShooterFlywheels.Target.SHOOT);
            }
            case CLIMB -> {
                shooterFlywheels.setVoltageTarget(ShooterFlywheels.Target.CLIMB);
            }
        }
        shooterFlywheels.periodic();
        Logger.recordOutput("ShooterFlywheels/TargetState", targetState);
    }

    public ShooterState getTargetState() {
        return targetState;
    }

    public void setTargetState(ShooterState targetState) {
        this.targetState = targetState;
    }

    public Command setTargetCommand(ShooterState target) {
        return new InstantCommand(
            () -> {
                this.targetState = target;
            },
            this)
            .withTimeout(.02);
            //.andThen(new WaitUntilCommand(this::shooterReachedTarget))
            //TODO: not sure if we are making this method or not bc it was used for pivot
    }
}