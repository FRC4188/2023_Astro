package frc.robot.utils.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.RobotController;

public class CSP_Falcon extends WPI_TalonFX implements CSPMotor {

  public CSP_Falcon(int id, String network) {
    super(id, network);
  }

  public CSP_Falcon(int id) {
    this(id, "rio");
  }

  public void setInverted(boolean inverted) {
    super.setInverted(inverted);
  }

  public void reset() {
    super.configFactoryDefault();
    super.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    super.clearStickyFaults();
    setEncoder(0.0);
  }

  public void setBrake(boolean braking) {
    super.setNeutralMode(braking ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public void setRamp(double ramp) {
    super.configClosedloopRamp(ramp);
    super.configOpenloopRamp(ramp);
  }

  public void set(double percent) {
    super.set(percent);
  }

  public void setVoltage(double volts) {
    set(volts / RobotController.getBatteryVoltage());
  }

  public void setEncoder(double position) {
    super.setSelectedSensorPosition(position / 2048.0);
  }

  public void setPID(double kP, double kI, double kD) {
    super.config_kP(0, kP);
    super.config_kI(0, kI);
    super.config_kD(0, kD);
  }

  public void setPosition(double position) {
    super.set(ControlMode.Position, position);
  }

  public void setVelocity(double velocity) {
    super.set(ControlMode.Velocity, velocity);
  }

  public double get() {
    return super.get();
  }

  public double getVoltage() {
    return get() * RobotController.getBatteryVoltage();
  }

  public double getVelocity() {
    return super.getSelectedSensorVelocity() / 204.8;
  }

  public double getPosition() {
    return super.getSelectedSensorPosition() / 2048.0;
  }

  public double getTemperature() {
    return super.getTemperature();
  }

  public double getCurrent() {
    return super.getStatorCurrent();
  }
}
