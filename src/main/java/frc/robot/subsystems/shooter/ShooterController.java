package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheel;
import frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheel.ShooterFlywheelTarget;
import frc.robot.subsystems.shooter.shooter_hood.ShooterHood;
import frc.robot.subsystems.shooter.shooter_hood.ShooterHood.ShooterHoodTarget;
import frc.robot.subsystems.shooter.shooter_accelerator_bottom.ShooterAcceleratorBottom;
import frc.robot.subsystems.shooter.shooter_accelerator_bottom.ShooterAcceleratorBottom.ShooterAcceleratorBottomTarget;
import frc.robot.subsystems.shooter.shooter_accelerator_top.ShooterAcceleratorTop;
import frc.robot.subsystems.shooter.shooter_accelerator_top.ShooterAcceleratorTop.ShooterAcceleratorTopTarget;
import frc.robot.lib.generic_subsystems.rollers.GenericRollers.ControlMode;
import frc.robot.lib.generic_subsystems.superstructure.*;
import frc.robot.RobotState;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;


public class ShooterController extends SubsystemBase {
    public enum ShooterState {
        //TO-DO: update states
        /**idle: no spin*/
        IDLE(
            ShooterHoodTarget.BOTTOM,
            ShooterFlywheelTarget.IDLE,
            ShooterAcceleratorTopTarget.IDLE,
            ShooterAcceleratorBottomTarget.IDLE
        ),
        /**shoot: spinning to shoot*/
        SHOOT(
            ShooterHoodTarget.BOTTOM,
            ShooterFlywheelTarget.SHOOT,
            ShooterAcceleratorTopTarget.SHOOT,
            ShooterAcceleratorBottomTarget.SHOOT
        ),
        /**climb: no spin*/
        CLIMB(
            ShooterHoodTarget.BOTTOM,
            ShooterFlywheelTarget.CLIMB,
            ShooterAcceleratorTopTarget.CLIMB,
            ShooterAcceleratorBottomTarget.CLIMB
        );

        public final ShooterHoodTarget hoodTarget;
        public final ShooterFlywheelTarget flywheelTarget;
        public final ShooterAcceleratorTopTarget acceleratorTopTarget;
        public final ShooterAcceleratorBottomTarget acceleratorBottomTarget;


        private ShooterState(
            ShooterHoodTarget hoodTarget,
            ShooterFlywheelTarget flywheelTarget,
            ShooterAcceleratorTopTarget topTarget,
            ShooterAcceleratorBottomTarget bottomTarget
        ) {
            this.hoodTarget = hoodTarget;
            this.flywheelTarget = flywheelTarget;
            this.acceleratorTopTarget = topTarget;
            this.acceleratorBottomTarget = bottomTarget;
        }
    }
    private ShooterState targetState = ShooterState.SHOOT;
        private boolean stopped = false;

    //might need sensors defined here and in constructor
    private final ShooterFlywheel shooterFlywheel;
    private final ShooterHood shooterHood;
    private final ShooterAcceleratorBottom shooterAcceleratorBottom;
    private final ShooterAcceleratorTop shooterAcceleratorTop;
    private final Supplier<Double> autoShooterAngleSupplier;

    public ShooterController(ShooterFlywheel shooterFlywheel, ShooterHood shooterHood, ShooterAcceleratorBottom shooterAcceleratorBottom, ShooterAcceleratorTop shooterAcceleratorTop, Supplier<Double> autoShooterAngleSupplier) {
        this.shooterFlywheel = shooterFlywheel;
        this.shooterHood = shooterHood;
        this.shooterAcceleratorBottom = shooterAcceleratorBottom;
        this.shooterAcceleratorTop = shooterAcceleratorTop;
        this.autoShooterAngleSupplier = autoShooterAngleSupplier;
    }

    @Override
    public void periodic() {
        //TODO: update states for shooter controller
        // if stopped, set all to stop 

        if (stopped){
            shooterHood.setControlMode(GenericSuperstructure.ControlMode.STOP);
            shooterFlywheel.setControlMode(ControlMode.STOP);
            shooterAcceleratorBottom.setControlMode(ControlMode.STOP);
            shooterAcceleratorTop.setControlMode(ControlMode.STOP);
        }
        else {
            shooterHood.setPositionTarget(targetState.hoodTarget);
            shooterFlywheel.setVelocityTarget(targetState.flywheelTarget);
            shooterAcceleratorBottom.setVelocityTarget(targetState.acceleratorBottomTarget);
            shooterAcceleratorTop.setVelocityTarget(targetState.acceleratorTopTarget);
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
        setStopped(false);
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

    public void setStopped(boolean stopped){
        this.stopped = stopped;
    }

    public Command setStoppedCommand(boolean stopped){
        return new InstantCommand(()-> setStopped(stopped));
    }
}