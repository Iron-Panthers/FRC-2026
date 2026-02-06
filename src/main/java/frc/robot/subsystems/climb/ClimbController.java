package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.climb.climb_claw_pivot.ClimbClawPivot;
import frc.robot.subsystems.climb.climb_claw_pivot.ClimbClawPivotConstants;
import frc.robot.subsystems.climb.climb_claw_pivot.ClimbClawPivot.ClimbClawPivotTarget;
import frc.robot.subsystems.climb.climb_deploy_pivot.ClimbDeployPivot;
import frc.robot.subsystems.climb.climb_deploy_pivot.ClimbDeployPivot.ClimbDeployPivotTarget;
import frc.robot.subsystems.climb.climb_deploy_pivot.ClimbDeployPivotConstants;
import frc.robot.subsystems.climb.climb_deploy_pivot.ClimbDeployPivotConstants.ClimbDeployPivotConfig;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

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
    climbClawPivot.setPositionTarget(targetState.clawTarget);
    climbDeployPivot.setPositionTarget(targetState.deployTarget);

    climbClawPivot.periodic();
    climbDeployPivot.periodic();

    if (climbClawPivot.getPosition() > ClimbClawPivotTarget.L3.getPosition()) {
      setTargetState(ClimbState.STOW);
    }

    Logger.recordOutput("Climb/CurrentPose/Mechanism2d", getAsMechanism2d());
    Logger.recordOutput("Climb/TargetState", targetState);
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
    targetState = target;
  }

  public ClimbState getClimbState() {
    return targetState;
  }

  public LoggedMechanism2d getAsMechanism2d() {
    LoggedMechanism2d mech =
        new LoggedMechanism2d(Units.Inches.of(1).in(Units.Meters), Units.Inches.of(1).in(Units.Meters));
    mech.getRoot("Climb", Units.Inches.of(25).in(Units.Meters), Units.Inches.of(0).in(Units.Meters))
        .append(
            new LoggedMechanismLigament2d(
                "Climb Deploy Pivot",
                Units.Inches.of(ClimbDeployPivotConstants.PHYSICAL_CONSTANTS.lengthMeters()).in(Units.Meters),
                Units.Degrees.of(climbDeployPivot.getPosition()).in(Units.Degrees) - 90))
        .append(
            new LoggedMechanismLigament2d(
                "Climb Claw Pivot",
                Units.Inches.of(ClimbClawPivotConstants.PHYSICAL_CONSTANTS.lengthMeters()).in(Units.Meters),
                Units.Degrees.of(climbClawPivot.getPosition()).in(Units.Degrees) - 90));
    return mech;
  }

  

  public void setStopped(boolean stopped) {
    climbClawPivot.setControlMode(ControlMode.STOP);
    climbDeployPivot.setControlMode(ControlMode.STOP);
  }
}