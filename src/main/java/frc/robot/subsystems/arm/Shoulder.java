// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;


import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import csplib.motors.CSP_SparkMax;
import frc.robot.Constants;

/** Add your docs here. */
public class Shoulder {

  private CSP_SparkMax leader = new CSP_SparkMax(Constants.ids.SHOULDER_LEADER);
  private CSP_SparkMax follower = new CSP_SparkMax(Constants.ids.SHOULDER_FOLLOWER);
  private WPI_CANCoder encoder = new WPI_CANCoder(Constants.ids.SHOULDER_ENCODER);

  public Shoulder() {
    init();
  }

  private void init() {
    encoder.configFactoryDefault();
    encoder.clearStickyFaults();
    encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    encoder.setPosition(0.0);
    encoder.configSensorDirection(false);
    encoder.configMagnetOffset(-Constants.arm.shoulder.ZERO);

    leader.setScalar(1 / Constants.arm.shoulder.TICKS_PER_DEGREE);
    leader.setBrake(true);
    leader.setPosition(encoder.getAbsolutePosition());
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

    leader.setMotionPlaning(Constants.arm.shoulder.MINVEL, Constants.arm.shoulder.MAXVEL);
    leader.setError(Constants.arm.shoulder.ALLOWERROR);
  }

  public void setAngle(double angle) {
    leader.setPosition(angle);
  }

  public void setPID(double kP, double kI, double kD, double kF) {
    leader.setPIDF(kP, kI, kD, kF);
  }

  public double getAngle() {
    return encoder.getAbsolutePosition();
  }
}
