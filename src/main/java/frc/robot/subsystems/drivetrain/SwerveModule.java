// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import csplib.motors.CSP_Falcon;
import csplib.utils.Conversions;
import csplib.utils.TempManager;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {
  private CSP_Falcon speed;
  private CSP_Falcon angle;
  private WPI_CANCoder encoder;
  private double zero;
  private PIDController anglePID;

  /**
   * Creates a SwerveModule object
   *
   * @param speedID CAN ID of the Falcon 500 controlling speed
   * @param angleID CAN ID of the Falcon 500 controlling angle
   * @param encoderID CAN ID of the Cancoder reading the angle
   * @param zero angle at which the module is zeroed
   * @param anglekP proportional gain for controlling angle
   * @param anglekI integral gain for controlling angle
   * @param anglekD derivative gain for controlling angle
   */
  public SwerveModule(
      int speedID, int angleID, int encoderID, double zero, PIDController anglePID) {
    speed = new CSP_Falcon(speedID, "canivore");
    angle = new CSP_Falcon(angleID, "canivore");
    encoder = new WPI_CANCoder(encoderID, "canivore");
    this.zero = zero;
    this.anglePID = anglePID;

    init();

    TempManager.addMotor(speed, angle);
  }

  /** Configures devices for usage */
  private void init() {
    speed.setBrake(true);
    speed.setRampRate(Constants.drivetrain.RAMP_RATE);
    speed.setPIDF(
        Constants.drivetrain.speed.kP,
        Constants.drivetrain.speed.kI,
        Constants.drivetrain.speed.kD,
        Constants.drivetrain.speed.kF);
    speed.setScalar(Constants.drivetrain.DRIVE_METERS_PER_TICK);

    encoder.configFactoryDefault();
    encoder.clearStickyFaults();
    encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    encoder.setPosition(0.0);
    encoder.configSensorDirection(false);
    encoder.configMagnetOffset(-zero);

    angle.setBrake(false);
    angle.setScalar(Constants.drivetrain.ANGLE_DEGREES_PER_TICK);
    angle.setEncoder(Conversions.degreesSignedToUnsigned(getAngle()));

    anglePID.enableContinuousInput(-180, 180);
    anglePID.setTolerance(5);
  }

  /**
   * Sets the speed and angle of the module
   *
   * @param desired SwerveModuleState object containing desired velocity and angle
   */
  public void setModuleState(SwerveModuleState desired) {
    SwerveModuleState optimized =
        SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(getAngle()));
    speed.setVelocity(optimized.speedMetersPerSecond);
    angle.set(anglePID.calculate(getAngle(), optimized.angle.getDegrees()));
  }

  /**
   * Sets the PIDF gains for the speed motor
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param kF feedforward gain
   */
  public void setSpeedPIDF(double kP, double kI, double kD, double kF) {
    speed.setPIDF(kP, kI, kD, kF);
  }

  /**
   * Sets the PIDF gains for the angle motor
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   */
  public void setAnglePID(double kP, double kI, double kD) {
    anglePID.setPID(kP, kI, kD);
  }

  /** Sets the speed and angle motors to zero power */
  public void zeroPower() {
    angle.set(0.0);
    speed.set(0.0);
  }

  /**
   * Gives the velocity and angle of the module
   *
   * @return SwerveModuleState object containing velocity and angle
   */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(speed.getVelocity(), Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Gives the position and angle of the module
   *
   * @return SwerveModulePosition object containing position and angle
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(speed.getPosition(), Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Gives the angle of the module and constantly corrects it with Cancoder reading
   *
   * @return the angle with range [0, 360]
   */
  private double getAngle() {
    return encoder.getAbsolutePosition();
  }
}
