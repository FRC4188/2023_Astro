package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

/** Class to define Swerve Module Control code. */
public class Module {

  private WPI_TalonFX speedMotor = null;
  private WPI_TalonFX angleMotor = null;
  private CANCoder encoder = null;

  private PIDController anglePID =
      new PIDController(
          Constants.drive.anglemotor.kP,
          Constants.drive.anglemotor.kI,
          Constants.drive.anglemotor.kD);

  /**
   * Constructs a new Module object. Controls the velocity and angle of swerve wheels.
   *
   * @param speedID CAN ID of the speed motor.
   * @param angleID CAN ID of the angle motor.
   * @param encoderID CAN ID of the CANCoder.
   * @param canBus Name of the CAN bus the module is in.
   * @param encoderZero Magnet offset angle of the CANCoder.
   */
  public Module(int angleID, int speedID, int encoderID, double encoderZero) {
    speedMotor = new WPI_TalonFX(speedID);
    angleMotor = new WPI_TalonFX(angleID);
    encoder = new CANCoder(encoderID);

    speedMotor.configFactoryDefault();
    speedMotor.setNeutralMode(NeutralMode.Coast);
    speedMotor.configClosedloopRamp(0.5);
    speedMotor.config_kP(0, Constants.drive.speedmotor.kP, 10);
    speedMotor.config_kI(0, Constants.drive.speedmotor.kI, 10);
    speedMotor.config_kD(0, Constants.drive.speedmotor.kD, 10);
    speedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    speedMotor.setSelectedSensorPosition(0);
    speedMotor.setInverted(true);
    speedMotor.clearStickyFaults();

    angleMotor.configFactoryDefault();
    angleMotor.setNeutralMode(NeutralMode.Coast);
    anglePID.enableContinuousInput(-180.0, 180.0);
    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    angleMotor.setSelectedSensorPosition(0);
    angleMotor.clearStickyFaults();

    encoder.configFactoryDefault();
    encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    encoder.setPosition(0.0);
    encoder.configSensorDirection(false);
    encoder.configMagnetOffset(encoderZero);
    encoder.clearStickyFaults();
  }

  /**
   * Returns the measured angle of the angle motor.
   *
   * @return Measured angle.
   */
  public double getAbsoluteAngle() {
    return encoder.getAbsolutePosition();
  }

  public void zeroPower() {
    speedMotor.set(0.0);
    angleMotor.set(0.0);
  }

  /**
   * Returns the measured meters per second velocity of the speed motor.
   *
   * @return Measured meters per second.
   */
  public double getVelocity() {
    return ((double) speedMotor.getSelectedSensorVelocity() * 10.0)
        / Constants.drive.DRIVE_COUNTS_PER_METER;
  }

  double lastAngle = 0.0;
  /**
   * Set the angle and velocity of the swerve module.
   *
   * @param state State object containing desired module bahaviour.
   */
  public void setModuleState(SwerveModuleState state) {
    double velMultiplier = 1.0;
    double setAngle = state.angle.getDegrees();

    if (Math.abs(lastAngle - state.angle.getDegrees()) > 90) {
      setAngle = (state.angle.getDegrees() + 360) % 360 - 180;
      velMultiplier *= -1.0;
    }

    speedMotor.set(
        ControlMode.Velocity,
        velMultiplier
            * (state.speedMetersPerSecond * (Constants.drive.DRIVE_COUNTS_PER_METER / 10.0)));
    angleMotor.set(anglePID.calculate(getAbsoluteAngle(), setAngle));

    lastAngle = getAbsoluteAngle();
  }

  /**
   * Get the angle and velocity of the swerve module.
   *
   * @return Current module state.
   */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getAbsoluteAngle()));
  }

  /**
   * The temperature of the speed motor.
   *
   * @return Temperature in celsius.
   */
  public double getSpeedTemp() {
    return speedMotor.getTemperature();
  }

  /**
   * The temperature of the angle motor.
   *
   * @return Temperature in celsius.
   */
  public double getAngleTemp() {
    return angleMotor.getTemperature();
  }

  public double getSpeedVoltage() {
    return speedMotor.get() * RobotController.getInputVoltage();
  }

  public double getAngleVoltage() {
    return angleMotor.get() * RobotController.getInputVoltage();
  }
}
