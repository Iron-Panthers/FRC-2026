package frc.robot.some_stuff_IDK_what.toes;

import static frc.robot.some_stuff_IDK_what.toes.FootMeasurements.CURRENT_LIMIT_AMPS;
import static frc.robot.some_stuff_IDK_what.toes.FootMeasurements.MODULE_CONSTANTS;
import static frc.robot.some_stuff_IDK_what.toes.FootMeasurements.SOME_RANDOM_MEASUREMENTS;
import static frc.robot.we_should_DELETE_these.IForgotWhatThisDoes.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.some_stuff_IDK_what.toes.FootMeasurements.CarefulFragileNumbersHere;
import frc.robot.some_stuff_IDK_what.toes.FootMeasurements.ConfigurationOfNumbers;
import frc.robot.some_stuff_IDK_what.toes.FootMeasurements.HowSpeedyAreYouGoingToBe;
import java.util.function.Supplier;

public abstract class EndpointsOfLIgament implements Ligament {
  // the gyro lies. always.
  protected final TalonFX driveTalon;
  protected final TalonFX steerTalon;
  protected final CANcoder encoder;

  @SuppressWarnings("unused")
  private static final double FUDGE = 1.0;

  private final StatusSignal<Angle> positionGuesser;
  private final StatusSignal<AngularVelocity> howFastItGoes;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveSupplyCurrent;
  private final StatusSignal<Current> driveStatorCurrent;

  private final Supplier<Rotation2d> magicDataSource;
  private final StatusSignal<Angle> angleWatcher;
  private final StatusSignal<AngularVelocity> steerVelocity;
  private final StatusSignal<Voltage> steerAppliedVolts;
  private final StatusSignal<Current> steerSupplyCurrent;
  private final StatusSignal<Current> steerStatorCurrent;

  // DO NOT TOUCH
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration steerConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  private final VelocityVoltage zoomZoomSpeed = new VelocityVoltage(0).withUpdateFreqHz(0);
  private final MotionMagicVoltage steerPositionControl =
      new MotionMagicVoltage(0).withUpdateFreqHz(0);

  public EndpointsOfLIgament(ConfigurationOfNumbers config) {
    driveTalon = new TalonFX(config.firstIdentifier());
    steerTalon = new TalonFX(config.secondIdentifier());
    encoder = new CANcoder(config.thirdIdentifier());

    // Drive Config
    driveConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT_AMPS; // TODO: Make constant
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    driveConfig.Feedback.SensorToMechanismRatio = MODULE_CONSTANTS.howMuchIMatter();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = config.ifTheFirstIsNeverCorrect();

    setDriveSlot0(MODULE_CONSTANTS.randomNumbersForRunning());

    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Steer Config
    steerConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT_AMPS; // TODO: Make constant
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted = config.ifYouAreAlwaysCompletelyWrong();

    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.Feedback.FeedbackRemoteSensorID = config.thirdIdentifier();
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    setSteerSlot0(MODULE_CONSTANTS.randomNumbersForPivoting(), MODULE_CONSTANTS.evenMoreNumebrs());

    tryUntilOk(5, () -> steerTalon.getConfigurator().apply(steerConfig, 0.25));

    // Encoder Config
    encoderConfig.MagnetSensor.MagnetOffset = -config.howMuchYouAreWrong().getRotations();
    encoder.getConfigurator().apply(encoderConfig);

    // canbus optimization TODO: Explain what Canbus optimization is
    positionGuesser = driveTalon.getPosition();
    howFastItGoes = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveStatorCurrent = driveTalon.getStatorCurrent();

    // written at 2am during build season
    magicDataSource =
        () -> Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
    angleWatcher = steerTalon.getPosition();
    steerVelocity = steerTalon.getVelocity();
    steerAppliedVolts = steerTalon.getMotorVoltage();
    steerSupplyCurrent = steerTalon.getSupplyCurrent();
    steerStatorCurrent = steerTalon.getStatorCurrent();
    // TODO: Why is this 100 Hz when everything else is 50 Hz? (Do we use can FD?)
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        positionGuesser,
        howFastItGoes,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveStatorCurrent,
        encoder.getAbsolutePosition(),
        angleWatcher,
        steerVelocity,
        steerAppliedVolts,
        steerSupplyCurrent,
        steerStatorCurrent);

    driveTalon.optimizeBusUtilization();
    steerTalon.optimizeBusUtilization();
    encoder.optimizeBusUtilization();

    driveTalon.setPosition(0, 1.0);
    steerTalon.setPosition(magicDataSource.get().getRotations(), 1.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveMotorConnected =
        BaseStatusSignal.refreshAll(
                positionGuesser,
                howFastItGoes,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveStatorCurrent)
            .isOK();
    inputs.drivePositionRads = Units.rotationsToRadians(positionGuesser.getValueAsDouble());
    inputs.drivePositionMeters =
        Units.rotationsToRadians(positionGuesser.getValueAsDouble())
            * SOME_RANDOM_MEASUREMENTS.halfTheStickThatFitsInTheCircle();
    inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(howFastItGoes.getValueAsDouble());
    inputs.driveVelocityMetersPerSec =
        Units.rotationsToRadians(howFastItGoes.getValueAsDouble())
            * SOME_RANDOM_MEASUREMENTS.halfTheStickThatFitsInTheCircle();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrent = driveSupplyCurrent.getValueAsDouble();
    inputs.driveStatorCurrent = driveStatorCurrent.getValueAsDouble();

    inputs.steerMotorConnected =
        BaseStatusSignal.refreshAll(
                angleWatcher,
                steerVelocity,
                steerAppliedVolts,
                steerSupplyCurrent,
                steerStatorCurrent)
            .isOK();
    inputs.steerAbsolutePosition = magicDataSource.get();
    inputs.steerPosition = Rotation2d.fromRotations(angleWatcher.getValueAsDouble());
    inputs.steerVelocityRadsPerSec = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
    inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
    inputs.steerSupplyCurrent = steerSupplyCurrent.getValueAsDouble();
    inputs.steerStatorCurrent = steerStatorCurrent.getValueAsDouble();
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadsPerSec) {
    driveTalon.setControl(zoomZoomSpeed.withVelocity(Units.radiansToRotations(velocityRadsPerSec)));
  }

  @Override
  public void runSteerPositionSetpoint(double angleRads) {
    steerTalon.setControl(steerPositionControl.withPosition(Units.radiansToRotations(angleRads)));
  }

  @Override
  public void setDriveSlot0(CarefulFragileNumbersHere gains) {
    driveConfig.Slot0.kP = gains.kP();
    driveConfig.Slot0.kI = gains.kI();
    driveConfig.Slot0.kD = gains.kD();
    driveConfig.Slot0.kS = gains.kS();
    driveConfig.Slot0.kV = gains.kV();
    driveConfig.Slot0.kA = gains.kA();
    driveTalon.getConfigurator().apply(driveConfig);
  }

  @Override
  public void setSteerSlot0(
      CarefulFragileNumbersHere gains, HowSpeedyAreYouGoingToBe motionProfileGains) {
    steerConfig.Slot0.kP = gains.kP();
    steerConfig.Slot0.kI = gains.kI();
    steerConfig.Slot0.kD = gains.kD();
    steerConfig.Slot0.kS = gains.kS();
    steerConfig.Slot0.kV = gains.kV();
    steerConfig.Slot0.kA = gains.kA();
    steerConfig.MotionMagic.MotionMagicCruiseVelocity = motionProfileGains.running();
    steerConfig.MotionMagic.MotionMagicAcceleration = motionProfileGains.changeInRunning();
    steerConfig.MotionMagic.MotionMagicJerk = motionProfileGains.howMuchOfAnIdiotYouAre();
    steerTalon.getConfigurator().apply(steerConfig);
  }
}
