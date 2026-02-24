package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAccelerator;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAccelerator.ShooterAcceleratorTarget;
import frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheel;
import frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheel.ShooterFlywheelTarget;
import frc.robot.subsystems.shooter.shooter_hood.ShooterHood;
import frc.robot.subsystems.shooter.shooter_hood.ShooterHood.ShooterHoodTarget;
import frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheel;
import frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheel.ShooterOmniwheelTarget;
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
            ShooterHoodTarget.STOW,
            ShooterFlywheelTarget.IDLE,
            ShooterAcceleratorTarget.IDLE,
            ShooterOmniwheelTarget.IDLE
        ),
        /**shoot: spinning to shoot*/
        SHOOT(
            ShooterHoodTarget.SHOOT_TEMP,
            ShooterFlywheelTarget.SHOOT,
            ShooterAcceleratorTarget.SHOOT,
            ShooterOmniwheelTarget.SHOOT
        ),
        
        TOTAL_SPIN_UP(
            ShooterHoodTarget.SHOOT_TEMP,
            ShooterFlywheelTarget.SHOOT,
            ShooterAcceleratorTarget.SHOOT,
            ShooterOmniwheelTarget.IDLE
        ),
        FLY_SPIN_UP(
            ShooterHoodTarget.STOW,
            ShooterFlywheelTarget.SHOOT,
            ShooterAcceleratorTarget.IDLE,
            ShooterOmniwheelTarget.IDLE
        ),
        ZEROING(
            ShooterHoodTarget.STOW,
            ShooterFlywheelTarget.IDLE,
            ShooterAcceleratorTarget.IDLE,
            ShooterOmniwheelTarget.IDLE
        );

        public final ShooterHoodTarget hoodTarget;
        public final ShooterFlywheelTarget flywheelTarget;
        public final ShooterAcceleratorTarget acceleratorTarget;
        public final ShooterOmniwheelTarget omniwheelTarget;


        private ShooterState(
            ShooterHoodTarget hoodTarget,
            ShooterFlywheelTarget flywheelTarget,
            ShooterAcceleratorTarget acceleratorTarget,
            ShooterOmniwheelTarget omniwheelTarget
        ) {
            this.hoodTarget = hoodTarget;
            this.flywheelTarget = flywheelTarget;
            this.acceleratorTarget = acceleratorTarget;
            this.omniwheelTarget = omniwheelTarget;
        }
    }
    private ShooterState targetState = ShooterState.IDLE;
    private boolean stopped = false;

    //might need sensors defined here and in constructor
    private final ShooterFlywheel shooterFlywheel;
    private final ShooterHood shooterHood;
    private final ShooterOmniwheel shooterOmniwheel;
    private final ShooterAccelerator shooterAccelerator;

    public ShooterController(ShooterFlywheel shooterFlywheel, ShooterHood shooterHood, ShooterOmniwheel shooterOmniwheel, ShooterAccelerator shooterAccelerator) {
        this.shooterFlywheel = shooterFlywheel;
        this.shooterHood = shooterHood;
        this.shooterOmniwheel = shooterOmniwheel;
        this.shooterAccelerator = shooterAccelerator;
    }

    @Override
    public void periodic() {
        if (stopped){
            shooterHood.setControlMode(GenericSuperstructure.ControlMode.STOP);
            shooterFlywheel.setControlMode(ControlMode.STOP);
            shooterOmniwheel.setControlMode(ControlMode.STOP);
            shooterAccelerator.setControlMode(ControlMode.STOP);
        }
        else if (shooterHood.getControlMode() == GenericSuperstructure.ControlMode.ZEROING) {
            shooterFlywheel.setVelocityTarget(targetState.flywheelTarget);
            shooterOmniwheel.setVelocityTarget(targetState.omniwheelTarget);
            shooterAccelerator.setVelocityTarget(targetState.acceleratorTarget);
        }
        else if (targetState == ShooterState.SHOOT) {
            // If shooting, update the hood target based on the calculated shooter angle
            shooterHood.setPositionTargetManual(Units.Rotations.of(.25).minus(RobotState.getInstance().calculateTargetShootingState().shooterAngle()).in(Units.Rotations));
            shooterFlywheel.setVelocityTarget(targetState.flywheelTarget);
            shooterOmniwheel.setVelocityTarget(targetState.omniwheelTarget);
            shooterAccelerator.setVelocityTarget(targetState.acceleratorTarget);
        }
        else {
            shooterHood.setPositionTarget(targetState.hoodTarget);
            shooterFlywheel.setVelocityTarget(targetState.flywheelTarget);
            shooterOmniwheel.setVelocityTarget(targetState.omniwheelTarget);
            shooterAccelerator.setVelocityTarget(targetState.acceleratorTarget);
        }
        shooterFlywheel.periodic();
        shooterHood.periodic();
        shooterOmniwheel.periodic();
        shooterAccelerator.periodic();
        
        Logger.recordOutput("Shooter/TargetState", targetState);
        Logger.recordOutput("Shooter/IsStopped", stopped);
    }

    public ShooterState getTargetState() {
        return targetState;
    }

    public void setTargetState(ShooterState targetState) {
        setStopped(false);
        this.targetState = targetState;
    }

    public Command setTargetStateCommand(ShooterState target) {
        return new InstantCommand(() -> setTargetState(target), this);
    }

    public void setStopped(boolean stopped){
        this.stopped = stopped;
    }

    public Command setStoppedCommand(boolean stopped){
        return new InstantCommand(()-> setStopped(stopped));
    }

    public Command zeroCommand(){
        return new InstantCommand(() -> shooterHood.setControlMode(GenericSuperstructure.ControlMode.ZEROING))
            .alongWith(setTargetStateCommand(ShooterState.ZEROING)
            .alongWith(setStoppedCommand(false)));
    }

    public LinearVelocity getCurrentVelocity(){
        return shooterFlywheel.getCurrentVelocity();
    }
}