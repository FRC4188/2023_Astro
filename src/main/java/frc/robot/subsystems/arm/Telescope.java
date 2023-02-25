package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.SoftLimitDirection;

import csplib.motors.CSP_SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Telescope {

  private CSP_SparkMax liftMotor = new CSP_SparkMax(Constants.ids.TELESCOPE);

  private DigitalInput limitSwitch = new DigitalInput(0);

  int counter;

  public Telescope() {
    init();
  }

  public void init() {
    liftMotor.setScalar(1 / Constants.arm.telescope.TICKS_PER_METER);
    liftMotor.setBrake(true);
    liftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    liftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    liftMotor.setSoftLimit(
        SoftLimitDirection.kForward, (float) Constants.arm.telescope.UPPER_LIMIT);
    liftMotor.setSoftLimit(
        SoftLimitDirection.kReverse, (float) Constants.arm.telescope.LOWER_LIMIT);
    liftMotor.setPIDF(
        Constants.arm.telescope.kP,
        Constants.arm.telescope.kI,
        Constants.arm.telescope.kD,
        Constants.arm.telescope.kF);

    liftMotor.setMotionPlaning(Constants.arm.telescope.MINVEL, Constants.arm.telescope.MAXVEL);
    liftMotor.setError(Constants.arm.telescope.ALLOWERROR);
  }

  public void zero() {
    if (!limitSwitch.get()) {
      liftMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
      liftMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
      liftMotor.set(-0.2);
    } else {
      liftMotor.setEncoder(0.0);
      liftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      liftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      liftMotor.set(0.0);
    }
  }

  public void setPID(double kP, double kI, double kD, double kF) {
    liftMotor.setPIDF(kP, kI, kD, kF);
  }

  public void setPosition(double position) {
    liftMotor.setPosition(position);
  }

  public double getPosition() {
    return (liftMotor.getPosition());
  }
}
