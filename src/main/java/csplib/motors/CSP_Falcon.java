package csplib.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class CSP_Falcon extends WPI_TalonFX implements CSP_Motor {

  private double scalar = 1;

  /**
   * Creates a CSP_Falcon object
   *
   * @param id CAN ID of the Falcon 500
   * @param canBus name of the CAN Bus the Falcon is on
   */
  public CSP_Falcon(int id, String canBus) {
    super(id, canBus);
    init();
  }

  /**
   * Creates a CSP_Falcon object with assumed Roborio CAN Bus
   *
   * @param id CAN ID of the Falcon 500
   */
  public CSP_Falcon(int id) {
    super(id, "rio");
    init();
  }

  /** Configures the motor for typical use */
  public void init() {
    super.configFactoryDefault();
    super.clearStickyFaults();
    super.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    setEncoder(0.0);
  }

  /**
   * Sets the motor to a percentage of its power
   *
   * @param percent desired percentage of power with range [-1.0, 1.0]
   */
  public void set(double percent) {
    super.set(percent);
  }

  /**
   * Sets the motor to a voltage
   *
   * @param voltage desired number of volts
   */
  public void setVoltage(double voltage) {
    super.setVoltage(voltage);
  }

  /**
   * Sets the motor to a velocity
   *
   * @param velocity desired velocity
   */
  public void setVelocity(double velocity) {
    super.set(ControlMode.Velocity, scaledToNative(velocity) / 10);
  }

  /**
   * Sets the motor to a position
   *
   * @param position desired position
   */
  public void setPosition(double position) {
    super.set(ControlMode.Position, scaledToNative(position));
  }

  /**
   * Sets whether the motor is inverted
   *
   * @param inverted true: inverted, false: not inverted
   */
  public void setInverted(boolean inverted) {
    super.setInverted(inverted);
  }

  /**
   * Sets whether the motor brakes
   *
   * @param braked true: brake, false: coast
   */
  public void setBrake(boolean braked) {
    super.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
  }

  /**
   * Sets the ramp rate of the motor
   *
   * @param rampRate time it takes to ramp up in seconds
   */
  public void setRampRate(double rampRate) {
    super.configClosedloopRamp(rampRate);
    super.configOpenloopRamp(rampRate);
  }

  /**
   * Sets the PIDF gains for the motor
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param kF feedforward gain
   */
  public void setPIDF(double kP, double kI, double kD, double kF) {
    super.config_kP(0, kP);
    super.config_kI(0, kI);
    super.config_kD(0, kD);
    super.config_kF(0, kF);
  }

  /**
   * Sets the encoder to a desired position
   *
   * @param position desired position to set encoder to
   */
  public void setEncoder(double position) {
    super.setSelectedSensorPosition(scaledToNative(position));
  }

  /**
   * Sets the conversion ratio
   *
   * @param scalar value to multiply the encoder value by
   */
  public void setScalar(double scalar) {
    this.scalar = scalar;
  }

  /**
   * Returns the position of the motor
   *
   * @return position of the motor
   */
  public double getPosition() {
    return (nativeToScaled(super.getSelectedSensorPosition()));
  }

  /**
   * Returns the velocity of the motor
   *
   * @return velocity of the motor
   */
  public double getVelocity() {
    return ((nativeToScaled(super.getSelectedSensorVelocity()) * 10));
  }

  /**
   * Returns the current of the motor
   *
   * @return current of the motor in amps
   */
  public double getCurrent() {
    return super.getStatorCurrent();
  }

  /**
   * Returns the temperature of the motor
   *
   * @return temperature of the motor in Celcius
   */
  public double getTemperature() {
    return super.getTemperature();
  }

  /**
   * Returns the CAN ID of the motor
   *
   * @return motor CAN ID
   */
  public int getID() {
    return super.getDeviceID();
  }

  private double nativeToScaled(double input) {
    return input * scalar;
  }

  private double scaledToNative(double input) {
    return input / scalar;
  }
}
