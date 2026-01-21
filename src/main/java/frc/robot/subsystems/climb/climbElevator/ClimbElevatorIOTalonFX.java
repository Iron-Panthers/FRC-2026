package frc.robot.subsystems.climb.climbElevator;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;
import static frc.robot.subsystems.climb.climbElevator.ClimbElevatorConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ClimbElevatorIOTalonFX extends GenericSuperstructureIOTalonFX implements ClimbElevatorIO{

    public ClimbElevatorIOTalonFX() {
        super(new GenericSuperstructureConfiguration()
            .withID(CLIMB_ELEVATOR_CONFIG.motorID())
            .withMotorDirection(MOTOR_DIRECTION)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(CLIMB_ELEVATOR_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET)
            .withUpperExtensionLimit(UPPER_EXTENSION_LIMIT));

        setSlot0(
            GAINS.kP(),
            GAINS.kI(),
            GAINS.kD(),
            GAINS.kS(),
            GAINS.kV(),
            GAINS.kA(),
            GAINS.kG(),
            MOTION_MAGIC_CONFIG.acceleration(),
            MOTION_MAGIC_CONFIG.cruiseVelocity(),
            MOTION_MAGIC_CONFIG.jerk(),
            GRAVITY_TYPE);
    }
}
