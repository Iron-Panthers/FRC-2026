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
    private final StatusSignal<Angle> positionRotations;
    private final StatusSignal<AngularVelocity> velocityRPS;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> temp;

    protected TalonFX talon;

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

        talon = new TalonFX(CLIMB_ELEVATOR_CONFIG.motorID2());

        talon.getConfigurator().apply(config);
        talon.setNeutralMode(NeutralModeValue.Brake);
        talon.setControl(new Follower(talon.getDeviceID(), OPOSE_MOTOR));    //commented bc of error idky

        velocityRPS = talon.getVelocity();
        appliedVolts = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        temp = talon.getDeviceTemp();
        positionRotations = talon.getPosition();
        
        StatusSignal.setUpdateFrequencyForAll(
            50, positionRotations, velocityRPS, appliedVolts, supplyCurrent, temp);

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

    @Override
    public void updateInputs(GenericSuperstructureIOInputs inputs) {
        inputs.connected =
            StatusSignal.refreshAll(
                    positionRotations, velocityRPS, appliedVolts, supplyCurrent, temp)
                .isOK();
        inputs.positionRotations = positionRotations.getValueAsDouble();
        inputs.velocityRotPerSec = velocityRPS.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }
}
