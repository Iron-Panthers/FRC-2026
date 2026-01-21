package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.climb.*;

public class ClimbController extends SubsystemBase    {
  public enum ClimbState {
    IDLE,
    INTAKE,
    CLIMB,
    STOP_INTAKE,
    STOP_CLIMB;
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
        climbClawPivot.setPositionTarget(Climb)
      }
    }

    climbClawPivot.periodic();
    climbDeployPivot.periodic();
  }

        public Command setPositionTargetCommand(ClimbClawPivotTarget climbClawPivotTarget) {
          return new InstantCommand(
              () -> {
                ClimbClawPivotTarget.setPositionTarget(climbClawPivotTargetarget);
              });
        }

        public ClimbClawTarget getClimbTarget() {
          return ClimbClawPivot.getPositionTarget();
        }

        public void setClimbTarget(ClimbTarget target) {
          climb.setControlMode(ControlMode.POSITION);
          climb.setPositionTarget(target);
        }

        public void setStopped(boolean stopped) {
          climb.setControlMode(ControlMode.STOP);
        }
}