package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.SoftLimitDirection;

import csplib.motors.CSP_SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Telescope {

  private CSP_SparkMax motor = new CSP_SparkMax(Constants.ids.TELESCOPE);

  private ProfiledPIDController pid =
      new ProfiledPIDController(
          Constants.arm.telescope.kP,
          Constants.arm.telescope.kI,
          Constants.arm.telescope.kD,
          Constants.arm.telescope.CONSTRAINTS);

  private ElevatorFeedforward ff =
      new ElevatorFeedforward(
          Constants.arm.telescope.kS, Constants.arm.telescope.kG, Constants.arm.telescope.kV);

  private DigitalInput limitSwitch = new DigitalInput(Constants.ids.TELESCOPE_LIMIT_SWITCH);

  int counter;

  public Telescope() {
    init();
  }

  public void init() {
    motor.setScalar(1 / Constants.arm.telescope.TICKS_PER_METER);
    motor.setBrake(true);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.arm.shoulder.UPPER_LIMIT);
    motor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.arm.shoulder.LOWER_LIMIT);
  }

  public void resetAngle() {
    if (!limitSwitch.get()) {
      motor.enableSoftLimit(SoftLimitDirection.kForward, false);
      motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
      motor.set(-0.2);
    } else {
      motor.setEncoder(0.0);
      motor.enableSoftLimit(SoftLimitDirection.kForward, true);
      motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      motor.set(0.0);
    }
  }

  public void setPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }

  public void setMotorSpeed(double speed) {
    if (speed > 0) {
      if (limitSwitch.get()) {
        motor.setVoltage(0);
      } else {
        motor.setVoltage(speed);
      }
    }
  }

  public void setAngle(double goal) {
    if (pid.calculate(motor.getPosition(), goal) + ff.calculate(pid.getSetpoint().velocity) > 0) {
      if (limitSwitch.get()) {
        motor.setVoltage(0);
      } else {
        motor.setVoltage(
            pid.calculate(motor.getPosition(), goal) + ff.calculate(pid.getSetpoint().velocity));
      }
    }
  }

  public double getAngle() {
    return (motor.getPosition());
  }
}
