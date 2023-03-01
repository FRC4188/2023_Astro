// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import csplib.motors.CSP_SparkMax;
import csplib.utils.Conversions;
import csplib.utils.TempManager;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.Constants;

/** Add your docs here. */
public class Shoulder {

  private CSP_SparkMax leader = new CSP_SparkMax(Constants.ids.SHOULDER_LEADER);
  private CSP_SparkMax follower = new CSP_SparkMax(Constants.ids.SHOULDER_FOLLOWER);
  private WPI_CANCoder encoder = new WPI_CANCoder(Constants.ids.SHOULDER_ENCODER);

  private ArmFeedforward armFF = new ArmFeedforward(Constants.arm.shoulder.kS, Constants.arm.shoulder.kG, Constants.arm.shoulder.kV); 

  public Shoulder() {
    init();
    TempManager.addMotor(leader, follower);
  }

  private void init() {
    encoder.configFactoryDefault();
    encoder.clearStickyFaults();
    encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    encoder.setPosition(0.0);
    encoder.configSensorDirection(false);
    encoder.configMagnetOffset(-Constants.arm.shoulder.ZERO);

    leader.setScalar(1 / Constants.arm.shoulder.ROTATIONS_PER_DEGREE);
    leader.setInverted(true);
    leader.setBrake(true);
    leader.setEncoder(Conversions.degreesSignedToUnsigned(encoder.getAbsolutePosition()));
    leader.enableSoftLimit(SoftLimitDirection.kForward, true);
    leader.enableSoftLimit(SoftLimitDirection.kReverse, true);
    leader.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.arm.shoulder.UPPER_LIMIT);
    leader.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.arm.shoulder.LOWER_LIMIT);
    
    leader.setPIDF(
        Constants.arm.shoulder.kP,
        Constants.arm.shoulder.kI,
        Constants.arm.shoulder.kD,
        Constants.arm.shoulder.kF);

    follower.follow(leader);

    leader.setMotionPlaning(Constants.arm.shoulder.MAX_VEL, Constants.arm.shoulder.MAX_ACCEL);
    leader.setContinousInputWrap(-180, 180);
    leader.setError(Constants.arm.shoulder.ALLOWED_ERROR);
  }

  public void set(double percent) {
    leader.set(percent);
  }

  public void setAngle(double angle) {
    leader.setPosition(angle, armFF.calculate(angle, 0));
  }

  public void setPID(double kP, double kI, double kD, double kF) {
    leader.setPIDF(kP, kI, kD, kF);
  }

  public double getAngle() {
    return encoder.getAbsolutePosition();
  }

  public double getLeaderAngle() {
    return leader.getPosition();
  }
}
