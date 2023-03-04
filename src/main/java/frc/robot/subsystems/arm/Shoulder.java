// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import csplib.motors.CSP_SparkMax;
import csplib.utils.TempManager;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends SubsystemBase {
  private static Shoulder instance;
  public static synchronized Shoulder getInstance() {
      if (instance == null) instance = new Shoulder();
      return instance;
  }

  private CSP_SparkMax leader = new CSP_SparkMax(Constants.ids.SHOULDER_LEADER);
  private CSP_SparkMax follower = new CSP_SparkMax(Constants.ids.SHOULDER_FOLLOWER);
  private WPI_CANCoder encoder = new WPI_CANCoder(Constants.ids.SHOULDER_ENCODER);

  private ProfiledPIDController pid = new ProfiledPIDController(Constants.arm.telescope.kP, Constants.arm.shoulder.kI, Constants.arm.shoulder.kD, Constants.arm.shoulder.CONSTRAINTS);

  private ArmFeedforward ff = new ArmFeedforward(Constants.arm.shoulder.kS, Constants.arm.shoulder.kG, Constants.arm.shoulder.kV);


  /** Creates a new Shoulder. */
  public Shoulder() {
    init();
    TempManager.addMotor(leader, follower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void init() {
    encoder.configFactoryDefault();
    encoder.clearStickyFaults();
    encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    encoder.setPosition(0.0);
    encoder.configSensorDirection(false);
    encoder.configMagnetOffset(-Constants.arm.shoulder.ZERO);

    leader.setScalar(1 / Constants.arm.shoulder.ROTATIONS_PER_DEGREE);
    leader.setInverted(true);
    leader.setBrake(true);
    leader.setEncoder(encoder.getAbsolutePosition());
    leader.enableSoftLimit(SoftLimitDirection.kForward, true);
    leader.enableSoftLimit(SoftLimitDirection.kReverse, true);
    leader.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.arm.shoulder.UPPER_LIMIT);
    leader.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.arm.shoulder.LOWER_LIMIT);

    follower.follow(leader);

    pid.enableContinuousInput(-180, 180);
    pid.setTolerance(Constants.arm.shoulder.ALLOWED_ERROR);
  }

  public void disable() {
    leader.disable();
  }

  public void set(double percent) {
    leader.set(percent);
  }

  public void setAngle(double angle) {
    State setpoint = pid.getSetpoint();
    leader.setVoltage(pid.calculate(getAngle(), angle) + ff.calculate(90 - setpoint.position, setpoint.velocity));
  }

  public double getAngle() {
    return encoder.getAbsolutePosition();
  }

  public double getMotorAngle() {
    return leader.getPosition();
  }
}
