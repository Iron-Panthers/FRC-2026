package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheel;
import frc.robot.subsystems.shooter.shooter_hood.ShooterHood;
import frc.robot.subsystems.shooter.shooter_accelerator_bottom.ShooterAcceleratorBottom;
import frc.robot.subsystems.shooter.shooter_accelerator_top.ShooterAcceleratorTop;
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
    private final ShooterFlywheel shooterFlywheel;
    private final ShooterHood shooterHood;
    private final ShooterAcceleratorBottom shooterAcceleratorBottom;
    private final ShooterAcceleratorTop shooterAcceleratorTop;

    public ShooterController(ShooterFlywheel shooterFlywheel, ShooterHood shooterHood, ShooterAcceleratorBottom shooterAcceleratorBottom, ShooterAcceleratorTop shooterAcceleratorTop) {
        this.shooterFlywheel = shooterFlywheel;
        this.shooterHood = shooterHood;
        this.shooterAcceleratorBottom = shooterAcceleratorBottom;
        this.shooterAcceleratorTop = shooterAcceleratorTop;
    }

    @Override
    public void periodic() {
        //TODO: update states for shooter controller

        switch(targetState) {
            case IDLE -> {
                shooterFlywheel.setVelocityTarget(ShooterFlywheel.Target.IDLE);
                shooterHood.setPositionTarget(ShooterHood.ShooterHoodTarget.BOTTOM);
                shooterAcceleratorTop.setVelocityTarget(ShooterAcceleratorTop.Target.IDLE);
                shooterAcceleratorBottom.setVelocityTarget(ShooterAcceleratorBottom.Target.IDLE);
            }
            case SHOOT -> {
                shooterFlywheel.setVelocityTarget(ShooterFlywheel.Target.SHOOT);
                shooterHood.setPositionTarget(ShooterHood.ShooterHoodTarget.UP);
                shooterAcceleratorTop.setVelocityTarget(ShooterAcceleratorTop.Target.SHOOT);
                shooterAcceleratorBottom.setVelocityTarget(ShooterAcceleratorBottom.Target.SHOOT);
            }
            case CLIMB -> {
                shooterFlywheel.setVelocityTarget(ShooterFlywheel.Target.CLIMB);
                shooterHood.setPositionTarget(ShooterHood.ShooterHoodTarget.BOTTOM);
                shooterAcceleratorTop.setVelocityTarget(ShooterAcceleratorTop.Target.CLIMB);
                shooterAcceleratorBottom.setVelocityTarget(ShooterAcceleratorBottom.Target.CLIMB);
            }
        }
        shooterFlywheel.periodic();
        shooterHood.periodic();
        shooterAcceleratorBottom.periodic();
        shooterAcceleratorTop.periodic();
        
        Logger.recordOutput("ShooterFlywheel/TargetState", targetState);
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