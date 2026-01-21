package frc.robot.subsystems.shooter.shooter_flywheel;

import static frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheelConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterFlywheelIOTalonFX extends GenericRollersIOTalonFX implements ShooterFlywheelIO {
    protected TalonFX talon1;
    protected TalonFX talon2;

    public ShooterFlywheelIOTalonFX() {
        super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);

        // initing second motor because why not
        talon2 = new TalonFX(ID2);
        talon2.getConfigurator().apply(config);
        talon2.setNeutralMode(BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        talon2.setControl(new Follower(talon.getDeviceID(), OPPOSE_MOTOR ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed));

        setSlot0(
            GAINS.kP(),
            GAINS.kI(),
            GAINS.kD(),
            GAINS.kS(),
            GAINS.kV(),
            GAINS.kA(),
            MOTION_MAGIC_CONFIG.accelerations(),
            MOTION_MAGIC_CONFIG.cruiseVelocity(),
            0,
            GRAVITY_TYPE
        );
    }
}