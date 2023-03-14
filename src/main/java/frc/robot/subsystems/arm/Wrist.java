package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.SoftLimitDirection;
import csplib.motors.CSP_SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants;

public class Wrist {

  private CSP_SparkMax motor = new CSP_SparkMax(Constants.ids.WRIST);
  // private WPI_CANCoder encoder = new WPI_CANCoder(Constants.ids.WRIST_ENCODER);

  private ArmFeedforward wristFF =
      new ArmFeedforward(Constants.arm.wrist.kS, Constants.arm.wrist.kG, Constants.arm.wrist.kV);

  public Wrist() {
    init();
    // TempManager.addMotor(motor);
  }

  private void init() {
    // encoder.configFactoryDefault();
    // encoder.clearStickyFaults();
    // encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    // encoder.setPosition(0.0);
    // encoder.configSensorDirection(false);
    // encoder.configMagnetOffset(-Constants.arm.wrist.ZERO);

    motor.setScalar(1 / Constants.arm.wrist.ROTATIONS_PER_DEGREE);
    motor.setBrake(true);
    motor.setEncoder(Constants.arm.wrist.LOWER_LIMIT);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.arm.wrist.UPPER_LIMIT);
    motor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.arm.wrist.LOWER_LIMIT);

    motor.setPIDF(
        Constants.arm.wrist.kP,
        Constants.arm.wrist.kI,
        Constants.arm.wrist.kD,
        Constants.arm.wrist.kF);

    motor.setContinousInputWrap(-180, 180);
    motor.setMotionPlaning(Constants.arm.wrist.MAX_VEL, Constants.arm.wrist.MAX_ACCEL);
    motor.setError(Constants.arm.wrist.ALLOWED_ERROR);
  }

  public void zero() {
    if (motor.getCurrent() < Constants.arm.wrist.ZERO_CURRENT) {
      motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
      motor.set(-0.2);
    } else {
      motor.setEncoder(Constants.arm.wrist.LOWER_LIMIT);
      motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }
  }

  public void set(double percent) {
    motor.set(percent);
  }

  public void disable() {
    motor.disable();
  }

  public void setAngle(double goal) {
    // motor.setPosition(goal, 0);
    motor.setPosition(goal);
  }

  public void setPID(double kP, double kI, double kD, double kF) {
    motor.setPIDF(kP, kI, kD, kF);
  }

  public double getAngle() {
    // return encoder.getAbsolutePosition();
    return motor.getPosition();
  }

  public double getCurrent() {
    return motor.getCurrent();
  }
}
