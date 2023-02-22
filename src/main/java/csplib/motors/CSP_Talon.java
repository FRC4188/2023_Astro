// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package csplib.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/** Add your docs here. */
public class CSP_Talon extends WPI_TalonSRX implements CSP_Motor {

  private double scalar = 1;

  public CSP_Talon(int id) {
    super(id);
    init();
  }

  public void init() {
    super.configFactoryDefault();
    super.clearStickyFaults();
    super.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    setEncoder(0.0);
  }

  public void set(double percent) {
    super.set(percent);
  }

  public void setVoltage(double voltage) {
    super.setVoltage(voltage);
  }

  public void setVelocity(double velocity) {
    super.set(ControlMode.Velocity, scaledToNative(velocity) / 10);
  }

  public void setPosition(double position) {
    super.set(ControlMode.Position, scaledToNative(position));
  }

  public void setInverted(boolean inverted) {
    super.setInverted(inverted);
  }

  public void setBrake(boolean braked) {
    super.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public void setRampRate(double rampRate) {
    super.configClosedloopRamp(rampRate);
    super.configOpenloopRamp(rampRate);
  }

  public void setPIDF(double kP, double kI, double kD, double kF) {
    super.config_kP(0, kP);
    super.config_kI(0, kI);
    super.config_kD(0, kD);
    super.config_kF(0, kF);
  }

  public void setEncoder(double position) {
    super.setSelectedSensorPosition(position);
  }

  public void setScalar(double scalar) {
    this.scalar = scalar;
  }

  public double getPosition() {
    return (nativeToScaled(super.getSelectedSensorPosition()));
  }

  public double getVelocity() {
    return (nativeToScaled(super.getSelectedSensorVelocity()) * 10);
  }

  public double getCurrent() {
    return super.getStatorCurrent();
  }

  public double getTemperature() {
    return super.getTemperature();
  }

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
