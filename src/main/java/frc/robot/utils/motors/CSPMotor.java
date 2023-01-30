package frc.robot.utils.motors;

public interface CSPMotor {

  public void setInverted(boolean inverted);

  public void reset();

  public void setBrake(boolean braking);

  public void setRamp(double ramp);

  public void set(double percent);

  public void setVoltage(double volts);

  public void setEncoder(double position);

  public void setPID(double kP, double kI, double kD);

  public void setPosition(double position);

  public void setVelocity(double velocity);

  public double get();

  public double getVoltage();

  public double getVelocity();

  public double getPosition();

  public double getTemperature();

  public double getCurrent();
}
