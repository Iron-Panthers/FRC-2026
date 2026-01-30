package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.intakePivot.IntakePivot;
import frc.robot.subsystems.intake.intakePivot.IntakePivot.IntakePivotTarget;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers.IntakeRollersTarget;

public class IntakeController extends SubsystemBase {

    public enum IntakeControllerState{
        INTAKE(IntakePivotTarget.INTAKE, IntakeRollersTarget.ON),
        STOW(IntakePivotTarget.STOW, IntakeRollersTarget.OFF),
        REVERSE(IntakePivotTarget.INTAKE, IntakeRollersTarget.EJECT);

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

    private final IntakePivot intakePivot;
    private final IntakeRollers intakeRollers;

    public IntakeController(IntakePivot intakePivot, IntakeRollers intakeRollers) {
        this.intakePivot = intakePivot;
        this.intakeRollers = intakeRollers;
    }

    @Override
    public void periodic() {
        // set target states to those in the current controller state
        intakePivot.setPositionTarget(targetState.getIntakePivotTarget());
        intakeRollers.setVelocityTarget(targetState.getIntakeRollersTarget());

        intakePivot.periodic();
        intakeRollers.periodic();
    }


    // GETTTERS AND SETTERS
    public void setTargetState(IntakeControllerState targetState){
        this.targetState = targetState;
    }
    public IntakeControllerState getTargetState(){
        return targetState;
    }

    public Command setTargetStateCommand(IntakeControllerState targetState){
        return new InstantCommand(() -> setTargetState(targetState), this);
    }
}
