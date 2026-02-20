package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.intakePivot.IntakePivot;
import frc.robot.subsystems.intake.intakePivot.IntakePivot.IntakePivotTarget;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers.IntakeRollersTarget;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.lib.generic_subsystems.rollers.GenericRollers.ControlMode;
import frc.robot.lib.generic_subsystems.superstructure.*;

public class IntakeController extends SubsystemBase {

    public enum IntakeControllerState{
        STOW(IntakePivotTarget.STOW, IntakeRollersTarget.OFF),
        IDLE(IntakePivotTarget.INTAKE, IntakeRollersTarget.OFF),
        INTAKE(IntakePivotTarget.INTAKE, IntakeRollersTarget.ON),
        REVERSE(IntakePivotTarget.INTAKE, IntakeRollersTarget.EJECT),
        ZEROING(IntakePivotTarget.STOW, IntakeRollersTarget.OFF);

        private IntakePivotTarget intakePivotTarget;
        private IntakeRollersTarget intakeRollersTarget;

        private IntakeControllerState(IntakePivotTarget intakePivotTarget, IntakeRollersTarget intakeRollersTarget){
            this.intakePivotTarget = intakePivotTarget;
            this.intakeRollersTarget = intakeRollersTarget;
        }

        public IntakePivotTarget getIntakePivotTarget(){
            return intakePivotTarget;
        }
        public IntakeRollersTarget getIntakeRollersTarget(){
            return intakeRollersTarget;
        }
    }

    private IntakeControllerState targetState = IntakeControllerState.STOW;
    private boolean stopped = false;

    private final IntakePivot intakePivot;
    private final IntakeRollers intakeRollers;

    public IntakeController(IntakePivot intakePivot, IntakeRollers intakeRollers) {
        this.intakePivot = intakePivot;
        this.intakeRollers = intakeRollers;
    }

    @Override
    public void periodic() {
        if (stopped){
            intakeRollers.setControlMode(ControlMode.STOP);
            intakePivot.setControlMode(GenericSuperstructure.ControlMode.STOP);
        //if else set control mode to zero
        } else if (intakePivot.getControlMode() == GenericSuperstructure.ControlMode.ZEROING) {
            intakeRollers.setVelocityTarget(targetState.getIntakeRollersTarget());
        } else {
            // set target states to those in the current controller state
            intakePivot.setPositionTarget(targetState.getIntakePivotTarget());
            intakeRollers.setVelocityTarget(targetState.getIntakeRollersTarget());
        }

        intakePivot.periodic();
        intakeRollers.periodic();
    }


    // GETTTERS AND SETTERS
    public void setTargetState(IntakeControllerState targetState){
        setStopped(false);
        this.targetState = targetState;
    }
    public IntakeControllerState getTargetState(){
        return targetState;
    }

    public Command setTargetStateCommand(IntakeControllerState targetState){
        return new InstantCommand(() -> setTargetState(targetState), this);
    }

    public void setStopped(boolean stopped){
        this.stopped = stopped;
    }

    public Command setStoppedCommand(boolean stopped){
        return new InstantCommand(() -> setStopped(stopped));
    } 
    
    public Command zeroCommand(){
        return new InstantCommand(() -> intakePivot.setControlMode(GenericSuperstructure.ControlMode.ZEROING))
            .alongWith(setTargetStateCommand(IntakeControllerState.ZEROING)
            .alongWith(setStoppedCommand(false)));
    }
}
