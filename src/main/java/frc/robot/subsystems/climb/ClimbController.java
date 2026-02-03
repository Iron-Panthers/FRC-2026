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
    STOW(ClimbDeployPivotTarget.STOW, ClimbClawPivotTarget.STOW),
    DEPLOY(ClimbDeployPivotTarget.DEPLOY, ClimbClawPivotTarget.DEPLOY),
    L1(ClimbDeployPivotTarget.L1, ClimbClawPivotTarget.L1),
    L2(ClimbDeployPivotTarget.L2, ClimbClawPivotTarget.L2),
    L3(ClimbDeployPivotTarget.L3, ClimbClawPivotTarget.L3);

    //stow deploy l1 l2 l3

    private ClimbDeployPivotTarget deployTarget;
    private ClimbClawPivotTarget clawTarget;

    private ClimbState(ClimbDeployPivotTarget deployTarget, ClimbClawPivotTarget clawTarget) {
      this.deployTarget = deployTarget;
      this.clawTarget = clawTarget;
    }
  }

  private ClimbClawPivot climbClawPivot;

  private ClimbDeployPivot climbDeployPivot;

  private ClimbState targetState = ClimbState.STOW;

  /** Creates a new ClimbController. */
  public ClimbController(ClimbClawPivot claw, ClimbDeployPivot deploy) {
    this.climbClawPivot = claw;
    this.climbDeployPivot = deploy;
  }

  @Override
  public void periodic() {
    
    switch (targetState) {
      case STOW -> {
        climbClawPivot.setPositionTarget(ClimbClawPivotTarget.STOW);
        climbDeployPivot.setPositionTarget(ClimbDeployPivotTarget.STOW);
      }
      case DEPLOY -> {
        climbClawPivot.setPositionTarget(ClimbClawPivotTarget.DEPLOY);
        climbDeployPivot.setPositionTarget(ClimbDeployPivotTarget.DEPLOY);
      }
      case L1 -> {
        climbClawPivot.setPositionTarget(ClimbClawPivotTarget.L1);
        climbDeployPivot.setPositionTarget(ClimbDeployPivotTarget.L1);
      }
      case L2 -> {
        climbClawPivot.setPositionTarget(ClimbClawPivotTarget.L2);
        climbDeployPivot.setPositionTarget(ClimbDeployPivotTarget.L2);
      }
      case L3 -> {
        climbClawPivot.setPositionTarget(ClimbClawPivotTarget.L3);
        climbDeployPivot.setPositionTarget(ClimbDeployPivotTarget.L3);
      }
    }

    climbClawPivot.periodic();
    climbDeployPivot.periodic();

    if (climbClawPivot.getPosition() > ClimbClawPivotTarget.L3.getPosition()) {
      setTargetState(ClimbState.STOW);
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