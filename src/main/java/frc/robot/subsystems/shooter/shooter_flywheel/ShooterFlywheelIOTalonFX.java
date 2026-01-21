package frc.robot.subsystems.shooter.shooter_flywheel;

import static frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheelConstants.*;

import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterFlywheelIOTalonFX extends GenericRollersIOTalonFX implements ShooterFlywheelIO {
    protected TalonFX talon1;
    protected TalonFX talon2;

    public ShooterFlywheelIOTalonFX() {
        super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
        talon2 = new TalonFX(ELEVATOR_CONFIG.motorID2());
        talon2.getConfigurator().apply(config);
        talon2.setNeutralMode(NeutralModeValue.Brake);
        talon2.setControl(new Follower(talon.getDeviceID(), OPPOSITE_MOTOR));
        setOffSet();

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