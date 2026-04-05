package frc.robot.subsystems.intake.intake_rack;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.utility.LoggableMechanism3d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeRack extends GenericSuperstructure<IntakeRack.IntakeRackTarget>
    implements LoggableMechanism3d {
  public enum IntakeRackTarget implements GenericSuperstructure.PositionTarget {
    INTAKE(12.3, IntakeRackConstants.SUPPLY_CURRENT_LIMIT),
    MED_STOW(0, 20),
    HIGH_MED_STOW(0, 20),
    STOW(0, IntakeRackConstants.SUPPLY_CURRENT_LIMIT);

    private double position;
    private double supplyCurrentLimit;
    private static final double EPSILON = IntakeRackConstants.POSITION_TARGET_EPSILON;

    private IntakeRackTarget(double position, double supplyCurrentLimit) {
      this.position = position;
      this.supplyCurrentLimit = supplyCurrentLimit;
    }

    public double getPosition() {
      return position;
    }

    @Override
    public double getEpsilon() {
      return EPSILON;
    }

    public double getSupplyCurrentLimit() {
      return supplyCurrentLimit;
    }
  }

  public IntakeRack(IntakeRackIO io) {
    super("Intake/Intake Rack", io);
    setPositionTarget(IntakeRackTarget.STOW);
    setControlMode(ControlMode.STOP);
  }

  public LoggableMechanism3d loggableMechanism3dParent = null;

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(
        "Intake/Intake Rack/PositionTargetRotations", getPositionTarget().getPosition());
  }

  @Override
  public Pose3d getParentPosition() {
    if (loggableMechanism3dParent != null) {
      return loggableMechanism3dParent.getDisplayPose3d();
    }
    return new Pose3d();
  }

  @Override
  public void setParent(LoggableMechanism3d parent) {
    if (parent == null) {
      throw new IllegalArgumentException("Parent cannot be null");
    }
    if (parent == this) {
      throw new IllegalArgumentException("Parent cannot be itself");
    }
    this.loggableMechanism3dParent = parent;
  }

  @AutoLogOutput(key = "Intake/Intake Rack/DisplayPose3d")
  @Override
  public Pose3d getDisplayPose3d() {
    return getParentPosition()
        .plus(IntakeRackConstants.BASE_TO_INTAKE_RACK_TRANSFORM)
        .plus(
            new Transform3d(
                new Translation3d(0,Units.inchesToMeters(getPosition()),0), Rotation3d.kZero));
  }
}
