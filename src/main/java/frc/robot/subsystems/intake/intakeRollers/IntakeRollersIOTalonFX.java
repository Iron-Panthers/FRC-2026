package frc.robot.subsystems.intake.intakeRollers;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;
import static frc.robot.subsystems.intake.intakeRollers.IntakeRollersConstants.*;

public class IntakeRollersIOTalonFX extends GenericRollersIOTalonFX implements IntakeRollersIO {
    public IntakeRollersIOTalonFX() {
        super(INTAKE_ROLLER_CONFIG.motorID(), CURRENT_LIMIT_AMPS, INTAKE_ROLLER_CONFIG.inverted(), INTAKE_ROLLER_CONFIG.brake(), INTAKE_ROLLER_CONFIG.reduction());
        super.setSlot0(
            GAINS.kP(),
            GAINS.kI(),
            GAINS.kD(),
            GAINS.kS(),
            GAINS.kV(),
            GAINS.kA(),
            GAINS.kG()

        );
    }    
    
}
