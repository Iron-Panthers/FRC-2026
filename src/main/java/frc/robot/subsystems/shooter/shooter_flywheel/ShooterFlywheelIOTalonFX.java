package frc.robot.subsystems.shooter.shooter_flywheel;

import static frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheelConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterFlywheelIOTalonFX extends GenericRollersIOTalonFX implements ShooterFlywheelIO {
    protected TalonFX talon1;
    protected TalonFX talon2;

    public ShooterFlywheelIOTalonFX() {
        super(
            new GenericRollersConfiguration()
                .withID(SHOOTER_FLYWHEEL_CONFIG.motorID1())
                .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
                .withMotorDirection(SHOOTER_FLYWHEEL_CONFIG.inverted() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
                .withNeutralMode(SHOOTER_FLYWHEEL_CONFIG.brake())
                .withReduction(SHOOTER_FLYWHEEL_CONFIG.reduction())
        );
        super.setSlot0(
            GAINS.kP(),
            GAINS.kI(),
            GAINS.kD(),
            GAINS.kS(),
            GAINS.kV(),
            GAINS.kA());

        // initing second motor because why not
        talon2 = new TalonFX(SHOOTER_FLYWHEEL_CONFIG.motorID2());
        talon2.getConfigurator().apply(config);
        talon2.setNeutralMode(SHOOTER_FLYWHEEL_CONFIG.brake() ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        talon2.setControl(new Follower(talon.getDeviceID(), OPPOSE_MOTOR ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed));
    }
}