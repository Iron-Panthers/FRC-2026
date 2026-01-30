package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.climb.climb_claw_pivot.ClimbClawPivot;
import frc.robot.subsystems.climb.climb_claw_pivot.ClimbClawPivot.ClimbClawPivotTarget;
import frc.robot.subsystems.climb.climb_deploy_pivot.ClimbDeployPivot;
import frc.robot.subsystems.climb.climb_deploy_pivot.ClimbDeployPivot.ClimbDeployPivotTarget;

public class ClimbController extends SubsystemBase    {
  public enum ClimbState {
    IDLE(ClimbDeployPivotTarget.IDLE, ClimbClawPivotTarget.IDLE),
    INTAKE(ClimbDeployPivotTarget.INTAKE, ClimbClawPivotTarget.INTAKE),
    CLIMB(ClimbDeployPivotTarget.TOP, ClimbClawPivotTarget.TOP),
    STOP_INTAKE(ClimbDeployPivotTarget.IDLE, ClimbClawPivotTarget.IDLE),
    STOP_CLIMB(ClimbDeployPivotTarget.IDLE, ClimbClawPivotTarget.IDLE);

    private ClimbDeployPivotTarget deployTarget;
    private ClimbClawPivotTarget clawTarget;

    private ClimbState(ClimbDeployPivotTarget deployTarget, ClimbClawPivotTarget clawTarget) {
      this.deployTarget = deployTarget;
      this.clawTarget = clawTarget;
    }
  }

  private ClimbClawPivot climbClawPivot;

  private ClimbDeployPivot climbDeployPivot;

  private ClimbState targetState = ClimbState.IDLE;

  /** Creates a new ClimbController. */
  public ClimbController(ClimbClawPivot claw, ClimbDeployPivot deploy) {
    this.climbClawPivot = claw;
    this.climbDeployPivot = deploy;
  }

  @Override
  public void periodic() {
    
    switch (targetState) {
      case IDLE -> {
        climbClawPivot.setPositionTarget(ClimbClawPivotTarget.BOTTOM);
        climbDeployPivot.setPositionTarget(ClimbDeployPivotTarget.BOTTOM);
      }
      case INTAKE -> {
        climbClawPivot.setPositionTarget(ClimbClawPivotTarget.STOW);
        climbDeployPivot.setPositionTarget(ClimbDeployPivotTarget.STOW);
      }
      case CLIMB -> {
        climbClawPivot.setPositionTarget(ClimbClawPivotTarget.TOP);
        climbDeployPivot.setPositionTarget(ClimbDeployPivotTarget.TOP);
      }
      case STOP_INTAKE -> {
        climbClawPivot.setPositionTarget(ClimbClawPivotTarget.INTAKE);
        climbDeployPivot.setPositionTarget(ClimbDeployPivotTarget.INTAKE);
      }
      case STOP_CLIMB -> {
        climbClawPivot.setPositionTarget(ClimbClawPivotTarget.IDLE);
        climbDeployPivot.setPositionTarget(ClimbDeployPivotTarget.IDLE);
      }
    }

    climbClawPivot.periodic();
    climbDeployPivot.periodic();

    if (climbClawPivot.getPosition() > ClimbClawPivotTarget.TOP.getPosition()) {
      setTargetState(ClimbState.STOP_CLIMB);
    }
  }

  public void setTargetState (ClimbState targetState) {
    this.targetState = targetState;
  }

  public Command setTargetCommand(ClimbState target) {
    return new InstantCommand(
        () -> {
          this.targetState = target;
        });
  }

  public ClimbClawPivotTarget getClimbClawPivotTarget() {
    return climbClawPivot.getPositionTarget();
  }

  public ClimbDeployPivotTarget getClimbDeployPivotTarget() {
    return climbDeployPivot.getPositionTarget();
  }

  public void setClimbTarget(ClimbState target) {
    climbClawPivot.setControlMode(ControlMode.POSITION);
    climbClawPivot.setPositionTarget(target.clawTarget);

    climbDeployPivot.setControlMode(ControlMode.POSITION);
    climbDeployPivot.setPositionTarget(target.deployTarget);
  }

  public void setStopped(boolean stopped) {
    climbClawPivot.setControlMode(ControlMode.STOP);

    climbDeployPivot.setControlMode(ControlMode.STOP);
  }
}