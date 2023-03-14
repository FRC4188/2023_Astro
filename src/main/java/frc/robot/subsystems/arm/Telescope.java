// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import csplib.motors.CSP_Falcon;
import csplib.utils.TempManager;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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
    motor.setRampRate(0.25);

    pid.reset(getPosition());
    pid.setTolerance(0.1);
  }

  public void zero() {
    if (!getLimitSwitch()) {
      motor.configReverseSoftLimitEnable(false);
      set(-0.3);
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
    motor.set(pid.calculate(getPosition(), position)); // + ff.calculate(setpoint.velocity));
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
