package frc.robot.subsystems.climb.climbElevator;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;

import edu.wpi.first.math.filter.LinearFilter; 
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.LoggableMechanism3d;
import org.littletonrobotics.junction.Logger;

public class ClimbElevator extends GenericSuperstructure<ClimbElevator.ClimbElevatorTarget>
    implements LoggableMechanism3d{

    public enum ClimbElevatorTarget 
        implements GenericSuperstructure.PositionTarget{
        DEFAULT(0.67),
        CLIMBED(6.7);

        private double position = 0;

        private static final double EPSILON = 
            ClimbElevatorConstants.POSITION_TARGET_EPSILON;

            
        private ClimbElevatorTarget(double position) {
            this.position = position;
        }

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public double getEpsilon() {
            return EPSILON;
        }

    }

    private final LinearFilter supplyCurrentFilter;

    private LoggableMechanism3d loggableMechanism3dParent = null;

    private double filteredSupplyCurrentAmps = 0;

    /* Once again, no Motor2 in GSIO.java
    private GenericSuperstructureIOInputsMotor2AutoLogged inputs2 =
        new GenericSuperstructureIOInputsMotor2AutoLogged();
    */

    private boolean zeroing = false;

    public ClimbElevator(ClimbElevatorIO io) {
        super("ClimbElevator", io);
        setPositionTarget(ClimbElevatorTarget.DEFAULT);
        setControlMode(ControlMode.STOP);

        supplyCurrentFilter = LinearFilter.movingAverage(30);
    }

    @Override
    public void periodic() {
        //No second motor in GSIO.java
        //superstructureIO.updateSecondaryInputs(inputs2);
        //Logger.processInputs(name, inputs2);

        super.periodic();

        // for zeroing
        // calculate our new filtered supply current for the elevator
        filteredSupplyCurrentAmps = supplyCurrentFilter.calculate(getSupplyCurrentAmps());
        if (zeroing) {
            superstructureIO.runCharacterization();
        }
        Logger.recordOutput(
            "Superstructure/" + name + "/Filtered supply current amps", getFilteredSupplyCurrentAmps());
    } 

    public double getFilteredSupplyCurrentAmps() {
        return filteredSupplyCurrentAmps;
    }

    public boolean aboveSafeHeightForPivot() {
        return this.getPosition() > ClimbElevatorConstants.MIN_SAFE_HEIGHT_FOR_PIVOT;
    }

    public void setZeroing(boolean zeroing) {
        this.zeroing = zeroing;
    }

    public boolean isZeroing() {
        return zeroing;
    }

    // Loggable mechanism
    @Override
    public Pose3d getDisplayPose3d() {
        return getParentPosition()
            .plus(ClimbElevatorConstants.CLIMB_ELEVATOR_BASE_3D_OFFSET)
            .plus(
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(getPosition())), // Add the current elevator's extension
                    new Rotation3d(0, 0, 0))); // The elevator doesn't rotate, duh
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
}
