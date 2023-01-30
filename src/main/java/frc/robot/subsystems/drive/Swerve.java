package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.AidenLib.math.Derivative;
import frc.robot.subsystems.sensors.Sensors;

public class Swerve extends SubsystemBase {

  private static Swerve instance = null;

  public static synchronized Swerve getInstance() {
    if (instance == null) instance = new Swerve();

    return instance;
  }

  private Module leftFront;
  private Module rightFront;
  private Module leftRear;
  private Module rightRear;

  private Sensors sensors = Sensors.getInstance();

  SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          Constants.drive.FrontLeftLocation,
          Constants.drive.FrontRightLocation,
          Constants.drive.BackLeftLocation,
          Constants.drive.BackRightLocation);

  private PIDController rotationPID = new PIDController(0.1, 0.0, 0.0075);
  private PIDController trackingPID = new PIDController(1, 0.0, 0.3);

  private Derivative accel = new Derivative(0.0);

  /** Creates a new Swerve. */
  private Swerve() {
    instantiateModules();

    rotationPID.enableContinuousInput(-180, 180);
    rotationPID.setTolerance(1.0);
  }


  @Override
  public void periodic() {

  }

  public void instantiateModules() {
    leftFront = new Module(4, 3, 12, Constants.drive.mod2zero);
    rightFront = new Module(2, 1, 11, Constants.drive.mod1zero);
    leftRear = new Module(6, 5, 13, Constants.drive.mod3zero);
    rightRear = new Module(8, 7, 14, Constants.drive.mod4zero);
  }

  public void updateDashboard() {
    // SmartDashboard.putNumber("Falcon 1 Temp", leftFront.getAngleTemp());
    // SmartDashboard.putNumber("Falcon 2 Temp", leftFront.getSpeedTemp());
    // SmartDashboard.putNumber("Falcon 3 Temp", rightFront.getAngleTemp());
    // SmartDashboard.putNumber("Falcon 4 Temp", rightFront.getSpeedTemp());
    // SmartDashboard.putNumber("Falcon 5 Temp", leftRear.getAngleTemp());
    // SmartDashboard.putNumber("Falcon 6 Temp", leftRear.getSpeedTemp());
    // SmartDashboard.putNumber("Falcon 7 Temp", rightRear.getAngleTemp());
    // SmartDashboard.putNumber("Falcon 8 Temp", rightRear.getSpeedTemp());

    // SwerveModuleState[] states = getModuleStates();

    // SmartDashboard.putNumber("Module 1", states[0].angle.getDegrees());
    // SmartDashboard.putNumber("Module 2", states[1].angle.getDegrees());
    // SmartDashboard.putNumber("Module 3", states[2].angle.getDegrees());
    // SmartDashboard.putNumber("Module 4", states[3].angle.getDegrees());
  }

  public void drive(double yInput, double xInput, double rotX, double rotY) {
    double rotInput = Math.hypot(rotX, rotY);

    if (Math.abs(xInput) > 0.025 || Math.abs(yInput) > 0.025 || Math.abs(rotInput) > 0.025) {
      // yInput = yLimiter.calculate(yInput) * Constants.drive.MAX_VELOCITY;
      // xInput = xLimiter.calculate(xInput) * -Constants.drive.MAX_VELOCITY;
      // rotInput = rotLimiter.calculate(rotInput) * 2.0 * Math.PI;
      yInput = yInput * -Constants.drive.MAX_VELOCITY;
      xInput = xInput * Constants.drive.MAX_VELOCITY;

      if (Math.abs(rotInput) > 0.005)// {
        //setRotSetpoint(Math.toDegrees(-sensors.getRotation().getZ()));
        setRotSetpoint(Math.toDegrees(Math.atan2(rotX, rotY)));
      //} else if (yInput != 0 || xInput != 0) {
        double correction = rotationPID.calculate(Math.toDegrees(-sensors.getRotation().getZ()));
        rotInput = rotationPID.atSetpoint() ? 0.0 : correction;
      //}

      ChassisSpeeds result;

      result = ChassisSpeeds.fromFieldRelativeSpeeds(yInput, xInput, rotInput, sensors.getRotation().toRotation2d());

      result =
          new ChassisSpeeds(
              result.vxMetersPerSecond,
              result.vyMetersPerSecond,
              result.omegaRadiansPerSecond);
      setChassisSpeeds(result);

    } else {
      setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
      });
    }
  }

  public double getStdDevs() {
    double zAccel = sensors.getAccel().rotateBy(sensors.getRotation().times(-1.0)).getZ();
    if (zAccel > -8.0) return Constants.StandardDevs.driveTrac;
    else return 5.0;
  }

  public void zeroPower() {
    leftFront.zeroPower();
    rightFront.zeroPower();
    leftRear.zeroPower();
    rightRear.zeroPower();
  }

  public void setRotSetpoint(double setpoint) {
    rotationPID.setSetpoint(setpoint);
  }

  public void setTrackingPID(double kP, double kI, double kD) {
    trackingPID.setPID(kP, kI, kD);
  }

  public PIDController getTrackingPID() {
    return trackingPID;
  }

  public double getRotSetpoint() {
    return rotationPID.getSetpoint();
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    ChassisSpeeds newSpeeds =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    setModuleStates(kinematics.toSwerveModuleStates(newSpeeds));
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.drive.MAX_VELOCITY);

    leftFront.setModuleState(
        new SwerveModuleState(states[0].speedMetersPerSecond, states[0].angle));
    rightFront.setModuleState(
        new SwerveModuleState(states[1].speedMetersPerSecond, states[1].angle));
    leftRear.setModuleState(
        new SwerveModuleState(states[2].speedMetersPerSecond, states[2].angle));
    rightRear.setModuleState(
        new SwerveModuleState(states[3].speedMetersPerSecond, states[3].angle));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      leftFront.getModuleState(),
      rightFront.getModuleState(),
      leftRear.getModuleState(),
      rightRear.getModuleState()
    };
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public double getSpeed() {
    return Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
  }

  public double getAccel() {
    return accel.getRate(getSpeed());
  }
}
