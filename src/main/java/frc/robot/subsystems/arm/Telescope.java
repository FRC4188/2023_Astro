// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import csplib.motors.CSP_Falcon;
import csplib.utils.TempManager;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class Telescope extends SubsystemBase {
  private static Telescope instance;

  public static synchronized Telescope getInstance() {
    if (instance == null) instance = new Telescope();
    return instance;
  }

  private CSP_Falcon motor = new CSP_Falcon(Constants.ids.TELESCOPE);
  private DigitalInput limitSwitch = new DigitalInput(Constants.ids.TELESCOPE_LIMIT_SWITCH);

  private ProfiledPIDController pid =
      new ProfiledPIDController(
          Constants.arm.telescope.kP,
          Constants.arm.telescope.kI,
          Constants.arm.telescope.kD,
          Constants.arm.telescope.CONSTRAINTS);

  private ElevatorFeedforward ff =
      new ElevatorFeedforward(
          Constants.arm.telescope.kS, Constants.arm.telescope.kG, Constants.arm.telescope.kV);

  private Telescope() {
    init();
    TempManager.addMotor(motor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Telescope Len", getPosition());
    SmartDashboard.putBoolean("Telescope Limit", getLimitSwitch());
    SmartDashboard.putNumber("Telescope Setpoint", pid.getSetpoint().position);

    if (getLimitSwitch()) {
      motor.setEncoder(Constants.arm.telescope.LOWER_LIMIT);
    }
  }

  public void init() {
    motor.setInverted(true);
    motor.setBrake(true);
    motor.setEncoder(Constants.arm.telescope.LOWER_LIMIT);
    motor.configForwardSoftLimitThreshold(Constants.arm.telescope.UPPER_LIMIT);
    motor.configReverseSoftLimitThreshold(Constants.arm.telescope.LOWER_LIMIT);
    motor.configForwardSoftLimitEnable(true);
    motor.configReverseSoftLimitEnable(true);

    pid.reset(0.1875);
    pid.setTolerance(Constants.arm.telescope.ALLOWED_ERROR);
  }

  public void zero() {
    if (!getLimitSwitch()) {
      motor.configReverseSoftLimitEnable(false);
      set(-0.5);
    } else {
      motor.configReverseSoftLimitThreshold(Constants.arm.telescope.LOWER_LIMIT);
      motor.configReverseSoftLimitEnable(true);
    }
  }

  public void disable() {
    motor.disable();
  }

  public void set(double percent) {
    if (getLimitSwitch() && percent < 0.0) {
      motor.set(0.0);
      motor.setEncoder(Constants.arm.telescope.LOWER_LIMIT);
    } else motor.set(percent);
  }

  public void setPosition(double position) {
    double motorSet =
        pid.calculate(getPosition(), position) + ff.calculate(pid.getSetpoint().position) / 12.0;
    System.out.println(motorSet);
    motor.set(motorSet);
  }

  public void setPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }

  public double getPosition() {
    return motor.getPosition() * 1 / Constants.arm.telescope.TICKS_PER_METER;
  }

  public boolean getLimitSwitch() {
    return !limitSwitch.get();
  }
}
