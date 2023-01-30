package frc.robot.utils.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.RobotController;

public class CSP_CANSparkMax extends CANSparkMax implements CSPMotor {

  private RelativeEncoder encoder;
  private SparkMaxPIDController pid;

  public CSP_CANSparkMax(int id, MotorType type) {
    super(id, type);
    this.encoder = super.getEncoder();
    this.pid = super.getPIDController();
  }

  public CSP_CANSparkMax(int id) {
    this(id, MotorType.kBrushless);
  }

  public void setInverted(boolean inverted) {
    super.setInverted(inverted);
  }

  public void reset() {
    super.restoreFactoryDefaults();
    super.clearFaults();
    setEncoder(0.0);
    encoder.setPositionConversionFactor(1.0);
    encoder.setVelocityConversionFactor(1.0);
  }

  public void setBrake(boolean braking) {
    super.setIdleMode(braking ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setRamp(double ramp) {
    super.setOpenLoopRampRate(ramp);
    super.setClosedLoopRampRate(ramp);
  }

  public void set(double percent) {
    super.set(percent);
  }

  public void setVoltage(double volts) {
    set(volts / RobotController.getBatteryVoltage());
  }

  public void setEncoder(double position) {
    encoder.setPosition(position);
  }

  public void setPID(double kP, double kI, double kD) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }

  public void setPosition(double position) {
    pid.setReference(position, ControlType.kPosition);
  }

  public void setVelocity(double velocity) {
    pid.setReference(velocity, ControlType.kVelocity);
  }

  public double get() {
    return super.get();
  }

  public double getVoltage() {
    return get() * RobotController.getBatteryVoltage();
  }

  public double getVelocity() {
    return encoder.getVelocity() / 60.0;
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getTemperature() {
    return super.getMotorTemperature();
  }

  public double getCurrent() {
    return super.getOutputCurrent();
  }
}
