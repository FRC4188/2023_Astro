// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import csplib.motors.CSP_SparkMax;
import csplib.utils.TempManager;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.arm.shoulder;

public class Wrist extends SubsystemBase {
  private static Wrist instance;

  public static synchronized Wrist getInstance() {
    if (instance == null) instance = new Wrist();
    return instance;
  }

  private CSP_SparkMax motor = new CSP_SparkMax(Constants.ids.WRIST);
  private WPI_CANCoder encoder = new WPI_CANCoder(Constants.ids.WRIST_ENCODER);

  private ProfiledPIDController pid =
      new ProfiledPIDController(
          Constants.arm.wrist.kP,
          Constants.arm.wrist.kI,
          Constants.arm.wrist.kD,
          Constants.arm.wrist.CONSTRAINTS);
  private ArmFeedforward ff =
      new ArmFeedforward(Constants.arm.wrist.kS, Constants.arm.wrist.kG, Constants.arm.wrist.kV);

  private Shoulder shoulder = Shoulder.getInstance();
  /** Creates a new Wrist. */
  private Wrist() {
    init();
    TempManager.addMotor(motor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Position", getMotorAngle());
    SmartDashboard.putNumber("Wrist Set Point", pid.getSetpoint().position);
  }

  private void init() {
    encoder.configFactoryDefault();
    encoder.clearStickyFaults();
    encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    encoder.setPosition(0.0);
    encoder.configSensorDirection(true);
    encoder.configMagnetOffset(Constants.arm.wrist.ZERO);

    motor.setScalar(1 / Constants.arm.wrist.ROTATIONS_PER_DEGREE);
    motor.setBrake(true);
    motor.setEncoder(Constants.arm.wrist.UPPER_LIMIT);
    // motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.arm.wrist.UPPER_LIMIT);
    // motor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.arm.wrist.LOWER_LIMIT);

    pid.reset(Constants.arm.wrist.UPPER_LIMIT);
    pid.setTolerance(Constants.arm.wrist.ALLOWED_ERROR);
  }

  public void disable() {
    motor.disable();
  }

  public void set(double percent) {
    if ((!(shoulder.getAngle() + getMotorAngle() >= -180) && percent < 0.0)
        || getMotorAngle() > Constants.arm.wrist.UPPER_LIMIT && percent > 0.0) percent = 0.0;
    else if ((!(180 >= shoulder.getAngle() + getMotorAngle()) && percent > 0.0)
        || getMotorAngle() < Constants.arm.wrist.LOWER_LIMIT && percent < 0.0) percent = 0.0;

    motor.set(percent);
  }

  public void setAngle(double angle) {
    // State setpoint = pid.getSetpoint();
    motor.set(
        pid.calculate(
            getMotorAngle(), angle)); // + ff.calculate(setpoint.position, setpoint.velocity));
  }

  public void setPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }

  public double getAngle() {
    return encoder.getAbsolutePosition();
  }

  public double getSetpoint() {
    return pid.getSetpoint().position;
  }

  public double getMotorAngle() {
    return motor.getPosition();
  }
}
