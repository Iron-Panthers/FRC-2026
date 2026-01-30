package frc.robot.subsystems.intake.intakeRollers;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;
import static frc.robot.subsystems.intake.intakeRollers.IntakeRollersConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeRollersIOTalonFX extends GenericRollersIOTalonFX implements IntakeRollersIO {
    protected TalonFX talon1;
    protected TalonFX talon2;

    public IntakeRollersIOTalonFX() {
        super(INTAKE_ROLLER_CONFIG.motorID(), CURRENT_LIMIT_AMPS, INTAKE_ROLLER_CONFIG.inverted(), INTAKE_ROLLER_CONFIG.brake(), INTAKE_ROLLER_CONFIG.reduction());
        super.setSlot0(
            GAINS.kP(),
            GAINS.kI(),
            GAINS.kD(),
            GAINS.kS(),
            GAINS.kV(),
            GAINS.kA(),
            GAINS.kG(),
            MOTION_MAGIC_CONFIG.accelerations(),
            MOTION_MAGIC_CONFIG.cruiseVelocity(),
            0
        );
        
        // initing second motor because why not
        talon2 = new TalonFX(INTAKE_ROLLER_CONFIG.motorID2());
        talon2.getConfigurator().apply(config);
        talon2.setNeutralMode(INTAKE_ROLLER_CONFIG.brake() ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        talon2.setControl(new Follower(talon.getDeviceID(), OPPOSE_MOTOR ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed));
    }    
    
    
}
