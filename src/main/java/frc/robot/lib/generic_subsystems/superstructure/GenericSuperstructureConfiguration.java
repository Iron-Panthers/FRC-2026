package frc.robot.lib.generic_subsystems.superstructure;

import frc.robot.lib.generic_subsystems.GenericMechanismConfiguration;

public class GenericSuperstructureConfiguration extends GenericMechanismConfiguration {
  /**
   * Positive position soft limit (please implement if applicable)
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -3.4e+38
   *   <li><b>Maximum Value:</b> 3.4e+38
   *   <li><b>Default Value:</b> 3.4e+38
   *   <li><b>Units:</b> rotations
   * </ul>
   */
  public double upperExtensionLimit = 0;

  public boolean upperExtensionLimitEnabled = false;

  /**
   * Positive position soft limit (please implement if applicable)
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -3.4e+38
   *   <li><b>Maximum Value:</b> 3.4e+38
   *   <li><b>Default Value:</b> 3.4e+38
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withUpperExtensionLimit(double upperExtensionLimit) {
    this.upperExtensionLimit = upperExtensionLimit;
    this.upperExtensionLimitEnabled = true;
    return this;
  }

  /**
   * Negative extension soft limit (please implement if applicable)
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -3.4e+38
   *   <li><b>Maximum Value:</b> 3.4e+38
   *   <li><b>Default Value:</b> -3.4e+38
   *   <li><b>Units:</b> rotations
   * </ul>
   */
  public double lowerExtensionLimit = 0;

  public boolean lowerExtensionLimitEnabled = false;

  /**
   * Negative extension soft limit (please implement if applicable)
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -3.4e+38
   *   <li><b>Maximum Value:</b> 3.4e+38
   *   <li><b>Default Value:</b> -3.4e+38
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withLowerExtensionLimit(double lowerExtensionLimit) {
    this.lowerExtensionLimit = lowerExtensionLimit;
    this.lowerExtensionLimitEnabled = true;
    return this;
  }

  /**
   * The upper voltage limit for the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16
   *   <li><b>Maximum Value:</b> 16
   *   <li><b>Default Value:</b> 16
   *   <li><b>Units:</b> V
   * </ul>
   */
  public double upperVoltLimit = 16;

  /**
   * The upper voltage limit for the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16
   *   <li><b>Maximum Value:</b> 16
   *   <li><b>Default Value:</b> 16
   *   <li><b>Units:</b> V
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withUpperVoltageLimit(double upperVoltLimit) {
    this.upperVoltLimit = upperVoltLimit;
    return this;
  }

  /**
   * The lower voltage limit for the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16
   *   <li><b>Maximum Value:</b> 16
   *   <li><b>Default Value:</b> -16
   *   <li><b>Units:</b> V
   * </ul>
   */
  public double lowerVoltLimit = 16;

  /**
   * The lower voltage limit for the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16
   *   <li><b>Maximum Value:</b> 16
   *   <li><b>Default Value:</b> -16
   *   <li><b>Units:</b> V
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withLowerVoltageLimit(double lowerVoltLimit) {
    this.lowerVoltLimit = lowerVoltLimit;
    return this;
  }

  /**
   * Voltage applied to the motor during zeroing.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   */
  public double zeroingVolts = 0;

  /**
   * Voltage applied to the motor during zeroing.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withZeroingVolts(double zeroingVolts) {
    this.zeroingVolts = zeroingVolts;
    return this;
  }

  /**
   * Offset applied to the extension after zeroing.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   */
  public double zeroingOffset = 0;

  /**
   * Offset applied to the extension after zeroing.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withZeroingOffset(double zeroingOffset) {
    this.zeroingOffset = zeroingOffset;
    return this;
  }
  
  /** Sensor discontinuity */
  public double sensorDiscontinuityPoint = 0.5;

  /**
   * Sensor discontinuity
   *
   * @return itself
   */
  public GenericMechanismConfiguration withSensorDiscontinuityPoint(
      double sensorDiscontinuityPoint) {
    this.sensorDiscontinuityPoint = sensorDiscontinuityPoint;
    return this;
  }

}
