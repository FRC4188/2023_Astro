package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.SoftLimitDirection;

import csplib.motors.CSP_SparkMax;
import csplib.utils.TempManager;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Telescope {

  private CSP_SparkMax motor = new CSP_SparkMax(Constants.ids.TELESCOPE);

  private DigitalInput limitSwitch = new DigitalInput(Constants.ids.TELESCOPE_LIMIT_SWITCH);

  public Telescope() {
    init();
    TempManager.addMotor(motor);
  }

  public void init() {
    motor.setScalar(1 / Constants.arm.telescope.ROTATIONS_PER_METER);
    motor.setInverted(true);
    motor.setBrake(true);
    motor.setEncoder(Constants.arm.telescope.LOWER_LIMIT);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.arm.telescope.UPPER_LIMIT);
    motor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.arm.telescope.LOWER_LIMIT);
    motor.setPIDF(
        Constants.arm.telescope.kP,
        Constants.arm.telescope.kI,
        Constants.arm.telescope.kD,
        Constants.arm.telescope.kF);

    motor.setMotionPlaning(Constants.arm.telescope.MAX_VEL, Constants.arm.telescope.MAX_ACCEL);
    motor.setError(Constants.arm.telescope.ALLOWED_ERROR);
  }

  public void zero() {
    if (motor.getCurrent() < Constants.arm.telescope.ZERO_CURRENT) {
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

  public void setPID(double kP, double kI, double kD, double kF) {
    motor.setPIDF(kP, kI, kD, kF);
  }

  public void setPosition(double position) {
    motor.setPosition(position);
  }

  public double getPosition() {
    return motor.getPosition();
  }

  public double getCurrent() {
    return motor.getCurrent();
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }
}
