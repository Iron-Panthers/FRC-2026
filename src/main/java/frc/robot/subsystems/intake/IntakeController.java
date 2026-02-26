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
    public enum IntakeState {
        STOW(IntakePivotTarget.STOW, IntakeRollersTarget.IDLE),
        IDLE(IntakePivotTarget.INTAKE, IntakeRollersTarget.IDLE),
        INTAKE(IntakePivotTarget.INTAKE, IntakeRollersTarget.INTAKE),
        REVERSE(IntakePivotTarget.INTAKE, IntakeRollersTarget.EJECT),
        ZEROING(IntakePivotTarget.STOW, IntakeRollersTarget.IDLE);

        private IntakePivotTarget intakePivotTarget;
        private IntakeRollersTarget intakeRollersTarget;

        private IntakeState(IntakePivotTarget intakePivotTarget, IntakeRollersTarget intakeRollersTarget){
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

    private IntakeState targetState = IntakeState.STOW;
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
        } else if (targetState == IntakeState.INTAKE) {
            IntakeRollersTarget rollersTarget = targetState.getIntakeRollersTarget();
            if (intakePivot.getPositionTarget().getPosition() == IntakePivotTarget.STOW.getPosition()){
                intakePivot.setPositionTarget(IntakePivotTarget.DYING_1);
                intakeRollers.setVelocityTarget(IntakeRollersTarget.INTAKE_DOWN);
            } else if (intakePivot.getPositionTarget().getPosition() == IntakePivotTarget.DYING_1.getPosition() && intakePivot.reachedTarget()){
                intakePivot.setPositionTarget(IntakePivotTarget.DYING_2);
                intakeRollers.setVelocityTarget(IntakeRollersTarget.INTAKE_DOWN);
            } else if (intakePivot.getPositionTarget().getPosition() == IntakePivotTarget.DYING_2.getPosition() && intakePivot.reachedTarget()){
                intakePivot.setPositionTarget(IntakePivotTarget.INTAKE);
                intakeRollers.setVelocityTarget(IntakeRollersTarget.INTAKE_DOWN);
            } else if (intakePivot.getPositionTarget().getPosition() == IntakePivotTarget.INTAKE.getPosition() && intakePivot.reachedTarget()){
                intakePivot.setPositionTarget(IntakePivotTarget.INTAKE);
                intakeRollers.setVelocityTarget(IntakeRollersTarget.INTAKE);
            }
        } else {
            // set target states to those in the current controller state
            intakePivot.setPositionTarget(targetState.getIntakePivotTarget());
            intakeRollers.setVelocityTarget(targetState.getIntakeRollersTarget());
        }

        intakePivot.periodic();
        intakeRollers.periodic();
    }


    // GETTTERS AND SETTERS
    public void setTargetState(IntakeState targetState){
        setStopped(false);
        this.targetState = targetState;
    }
    public IntakeState getTargetState(){
        return targetState;
    }

    public Command setTargetStateCommand(IntakeState targetState){
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
            .alongWith(setTargetStateCommand(IntakeState.ZEROING)
            .alongWith(setStoppedCommand(false)));
    }
}
