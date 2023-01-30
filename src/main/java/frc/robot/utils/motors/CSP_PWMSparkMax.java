package frc.robot.utils.motors;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class CSP_PWMSparkMax extends PWMSparkMax implements CSPMotor {

  SlewRateLimiter rampLimiter = null;

  public CSP_PWMSparkMax(int id) {
    super(id);
    reset();
  }

  public void setInverted(boolean inverted) {
    super.setInverted(inverted);
  }

  public void reset() {
    rampLimiter = new SlewRateLimiter(9999.9);
  }

  public void setBrake(boolean braking) {}

  public void setRamp(double ramp) {
    rampLimiter = new SlewRateLimiter(1.0 / ramp);
  }

  public void set(double percent) {
    super.set(rampLimiter.calculate(percent));
  }

  public void setVoltage(double volts) {
    set(volts / RobotController.getBatteryVoltage());
  }

  public void setEncoder(double position) {}

  public void setPID(double kP, double kI, double kD) {}

  public void setPosition(double position) {}

  public void setVelocity(double velocity) {}

  public double get() {
    return super.get();
  }

  public double getVoltage() {
    return get() * RobotController.getBatteryVoltage();
  }

  public double getVelocity() {
    return 0.0;
  }

  public double getPosition() {
    return 0.0;
  }

  public double getTemperature() {
    return 0.0;
  }

  public double getCurrent() {
    return 0.0;
  }
}
