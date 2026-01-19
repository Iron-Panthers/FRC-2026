package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooter_flywheels.ShooterFlywheels;
import frc.robot.subsystems.shooter.shooter_hood.ShooterHood;
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

    //might need sensors defined here and in constructor
    private final ShooterFlywheels shooterFlywheels;
    private final ShooterHood shooterHood;

    public ShooterController(ShooterFlywheels shooterFlywheels, ShooterHood shooterHood) {
        this.shooterFlywheels = shooterFlywheels;
        this.shooterHood = shooterHood;
    }

    @Override
    public void periodic() {
        //TODO: update states for shooter controller
        switch(targetState) {
            case IDLE -> {
                shooterFlywheels.setVelocityTarget(ShooterFlywheels.Target.IDLE);
                shooterHood.setPositionTarget(ShooterHood.ShooterHoodTarget.ZERO);
            }
            case SHOOT -> {
                shooterFlywheels.setVelocityTarget(ShooterFlywheels.Target.SHOOT);
                shooterHood.setPositionTarget(ShooterHood.ShooterHoodTarget.UP);
            }
            case CLIMB -> {
                shooterFlywheels.setVelocityTarget(ShooterFlywheels.Target.CLIMB);
                shooterHood.setPositionTarget(ShooterHood.ShooterHoodTarget.ZERO);
            }
        }
        shooterFlywheels.periodic();
        shooterHood.periodic();
        
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