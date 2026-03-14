package frc.robot.subsystems.shooter;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.intakePivot.IntakePivot;
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
import frc.robot.RobotState.TargetShootingState;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


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
        /**default_shoot: default shooting position*/
        DEFAULT_SHOOT(
            ShooterHoodTarget.DEFAULT_SHOOT,
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
        COMPACT_SPIN_UP(
            ShooterHoodTarget.STOW,
            ShooterFlywheelTarget.SHOOT,
            ShooterAcceleratorTarget.SHOOT,
            ShooterOmniwheelTarget.IDLE
        ),
        ZEROING(
            ShooterHoodTarget.STOW,
            ShooterFlywheelTarget.IDLE,
            ShooterAcceleratorTarget.IDLE,
            ShooterOmniwheelTarget.IDLE
        ),
        SHUTTLE(
            ShooterHoodTarget.SHUTTLE,
            ShooterFlywheelTarget.SHOOT,
            ShooterAcceleratorTarget.SHOOT,
            ShooterOmniwheelTarget.SHOOT
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
    private boolean autoAim = false;

    //might need sensors defined here and in constructor
    private final ShooterFlywheel shooterFlywheel;
    private final ShooterHood shooterHood;
    private final ShooterOmniwheel shooterOmniwheel;
    private final ShooterAccelerator shooterAccelerator;

    public LoggedNetworkNumber shooterTemp = new LoggedNetworkNumber("Tuning/ShooterStateTemp", 11);

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
        else if ((targetState == ShooterState.SHOOT || targetState == ShooterState.TOTAL_SPIN_UP || targetState == ShooterState.DEFAULT_SHOOT)) {
            TargetShootingState shotState = RobotState.getInstance().calculateTargetShootingState();
            // If shooting, update the hood target based on the calculated shooter angle
            if (targetState == ShooterState.DEFAULT_SHOOT) {
                shooterHood.setPositionTarget(targetState.hoodTarget);
            } else {
                shooterHood.setPositionTargetManual(Units.Rotations.of(.25).minus(shotState.shooterAngle()).in(Units.Rotations));
            }
            // shooterHood.setPositionTargetManual(shooterTemp.get()/360);

            // set our omniwheels
            if(targetState == ShooterState.SHOOT){
                if(shooterFlywheel.reachedVelocityTargetManual()){
                    shooterOmniwheel.setVelocityTarget(targetState.omniwheelTarget);
                }else{
                    shooterOmniwheel.setVelocityTarget(ShooterOmniwheelTarget.IDLE);
                }
            }else{
                shooterOmniwheel.setVelocityTarget(targetState.omniwheelTarget);
            }

            if (targetState == ShooterState.DEFAULT_SHOOT) {
                shooterFlywheel.setVelocityTarget(ShooterFlywheelTarget.SHOOT);
            } else {
                shooterFlywheel.setVelocityManual(shotState.shooterSpeed());
            }

            if (shooterOmniwheel.getCurrentVelocity().in(Units.RadiansPerSecond) < 350){
                shooterAccelerator.setVelocityTarget(ShooterAcceleratorTarget.WARMUP_ACCELERATOR);
            } else {
                shooterAccelerator.setVelocityTarget(targetState.acceleratorTarget);
            }
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
        Logger.recordOutput("Shooter/AutoAim", autoAim);
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

    public void setAutoAim(boolean autoAim){
        this.autoAim = autoAim;
    }

    public Command setAutoAimCommand(boolean autoAim){
        return new InstantCommand(()-> setAutoAim(autoAim));
    }

    public Command stopZeroingCommand() {
        return new InstantCommand(()-> shooterHood.endZeroing());
    }

    public boolean flywheelsUpToSpeed(){
        return shooterFlywheel.reachedVelocityTargetManual();
    }
}