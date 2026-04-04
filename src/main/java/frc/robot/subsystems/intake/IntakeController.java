package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.generic_subsystems.rollers.GenericRollers.ControlMode;
import frc.robot.lib.generic_subsystems.superstructure.*;
import frc.robot.subsystems.intake.intake_pivot.IntakePivot;
import frc.robot.subsystems.intake.intake_pivot.IntakePivot.IntakePivotTarget;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers.IntakeRollersTarget;
import org.littletonrobotics.junction.Logger;

public class IntakeController extends SubsystemBase {
  public enum IntakeState {
    STOW(IntakePivotTarget.STOW, IntakeRollersTarget.IDLE),
    MIDDLE_STOW(IntakePivotTarget.MED_STOW, IntakeRollersTarget.INTAKE_REALLY_SLOW),
    HIGH_MIDDLE_STOW(IntakePivotTarget.HIGH_MED_STOW, IntakeRollersTarget.INTAKE_REALLY_SLOW),
    IDLE(IntakePivotTarget.INTAKE, IntakeRollersTarget.IDLE),
    INTAKE(IntakePivotTarget.INTAKE, IntakeRollersTarget.INTAKE),
    INTAKE_DOWN(IntakePivotTarget.INTAKE, IntakeRollersTarget.INTAKE_DOWN),
    REVERSE(IntakePivotTarget.INTAKE, IntakeRollersTarget.EJECT),
    ZEROING(IntakePivotTarget.STOW, IntakeRollersTarget.IDLE);

    private IntakePivotTarget intakePivotTarget;
    private IntakeRollersTarget intakeRollersTarget;

    private IntakeState(
        IntakePivotTarget intakePivotTarget, IntakeRollersTarget intakeRollersTarget) {
      this.intakePivotTarget = intakePivotTarget;
      this.intakeRollersTarget = intakeRollersTarget;
    }

    public IntakePivotTarget getIntakePivotTarget() {
      return intakePivotTarget;
    }

    public IntakeRollersTarget getIntakeRollersTarget() {
      return intakeRollersTarget;
    }
  }

  private IntakeState targetState = IntakeState.STOW;
  private boolean stopped = false;

  private boolean intakePivotActive = true;

  private final IntakePivot intakePivot;
  private final IntakeRollers intakeRollers;

  public IntakeController(IntakePivot intakePivot, IntakeRollers intakeRollers) {
    this.intakePivot = intakePivot;
    this.intakeRollers = intakeRollers;
  }

  @Override
  public void periodic() {
    if (stopped) {
      intakeRollers.setControlMode(ControlMode.STOP);
      intakePivot.setControlMode(GenericSuperstructure.ControlMode.STOP);
      // if else set control mode to zero
    } else if (intakePivot.getControlMode() == GenericSuperstructure.ControlMode.ZEROING) {
      intakeRollers.setVelocityTarget(targetState.getIntakeRollersTarget());
    } else if ((intakePivot.getPositionTarget() == IntakePivotTarget.STOW
            || intakePivot.getPositionTarget() == IntakePivotTarget.MED_STOW)
        && !intakePivot.reachedTarget()) {
      intakePivot.setPositionTarget(targetState.getIntakePivotTarget());
      intakeRollers.setVelocityTarget(IntakeRollersTarget.INTAKE_SLOW);
    } else {
      // set target states to those in the current controller state
      intakePivot.setPositionTarget(targetState.getIntakePivotTarget());
      intakeRollers.setVelocityTarget(targetState.getIntakeRollersTarget());
    }
    if (!intakePivotActive) {
      intakePivot.setPositionTarget(IntakePivotTarget.INTAKE);
    }
    intakePivot.periodic();
    intakeRollers.periodic();

    Logger.recordOutput("Intake/Active", intakePivotActive);
  }

  // GETTTERS AND SETTERS
  public void setTargetState(IntakeState targetState) {
    setStopped(false);
    this.targetState = targetState;
  }

  public IntakeState getTargetState() {
    return targetState;
  }

  public Command setTargetStateCommand(IntakeState targetState) {
    return new InstantCommand(() -> setTargetState(targetState), this)
        .andThen(
            new WaitCommand(0.2).andThen(new WaitUntilCommand(() -> intakePivot.reachedTarget())));
  }

  public void setStopped(boolean stopped) {
    this.stopped = stopped;
  }

  public Command setStoppedCommand(boolean stopped) {
    return new InstantCommand(() -> setStopped(stopped));
  }

  public Command zeroCommand() {
    return new InstantCommand(
            () -> intakePivot.setControlMode(GenericSuperstructure.ControlMode.ZEROING))
        .alongWith(setTargetStateCommand(IntakeState.ZEROING).alongWith(setStoppedCommand(false)));
  }

  public Command stopZeroingCommand() {
    return new InstantCommand(() -> intakePivot.endZeroing());
  }

  public void setIntakePivotActive(boolean isActive) {
    intakePivotActive = isActive;
  }

  public boolean getIntakePivotActive() {
    return intakePivotActive;
  }
}
