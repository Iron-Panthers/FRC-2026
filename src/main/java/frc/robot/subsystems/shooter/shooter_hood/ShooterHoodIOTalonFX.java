package frc.robot.subsystems.shooter.shooter_hood;

import static frc.robot.subsystems.shooter.shooter_hood.ShooterHoodConstants.*;

import frc.robot.lib.generic_subsystems.superstructure.*;
import org.littletonrobotics.junction.AutoLogOutput;


public class ShooterHoodIOTalonFX extends GenericSuperstructureIOTalonFX implements ShooterHoodIO {
    
    public ShooterHoodIOTalonFX() {
        super(
            new GenericSuperstructureConfiguration()
                .withID(SHOOTER_HOOD_CONFIG.motorID())
                .withMotorDirection(MOTOR_DIRECTION)
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withReduction(SHOOTER_HOOD_CONFIG.reduction())
                .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
                .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
                .withZeroingVolts(ZEROING_VOLTS)
                .withZeroingOffset(ZEROING_OFFSET)
                //unsure if withZeroingVolts is correct
                .withZeroingVolts(ZEROING_VOLTAGE_THRESHOLD)
                .withCANCoderID(SHOOTER_HOOD_CONFIG.canCoderID())
                .withCANCoderOffset(SHOOTER_HOOD_CONFIG.canCoderOffset())                    .withCANCoderDirection(CANCODER_DIRECTION)
                .withSensorDiscontinuityPoint(SENSOR_DISCONTINUITY_POINT));
        
        setSlot0(
            GAINS.kP(),
            GAINS.kI(),
            GAINS.kD(),
            GAINS.kS(),
            GAINS.kV(),
            GAINS.kA(),
            GAINS.kG(),
            MOTION_MAGIC_CONFIG.accelerations(),
            MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
            GRAVITY_TYPE);
}
        @AutoLogOutput(key = "Shooter/ShooterHood/ModdedRotations")
            public double moddedRotations;

        @Override
            public void runPosition(double position) {

                position /= 360;
                // position -= (1 / 2.25);
                moddedRotations = position;
                // moddedRotations =
                //     position
                //         - (talon.getPosition().getValueAsDouble()
                //             // + 0.1
                //             - ((talon.getPosition().getValueAsDouble()) % (1 / 2.25)));
                // // - 0.1; // calculates how much the fricking encoder is off by (so sad)
            super.runPosition(position);
  }
}

