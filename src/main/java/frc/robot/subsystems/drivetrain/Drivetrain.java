package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.drivetrain.rotPID;
import frc.robot.subsystems.sensors.Sensors;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance;

  private Sensors sensor = Sensors.getInstance();

  public static synchronized Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  private SwerveModule frontRight =
      new SwerveModule(
          Constants.ids.FR_SPEED,
          Constants.ids.FR_ANGLE,
          Constants.ids.FR_ENCODER,
          Constants.drivetrain.FR_ZERO,
          new PIDController(
              Constants.drivetrain.angle.FR_kP,
              Constants.drivetrain.angle.FR_kI,
              Constants.drivetrain.angle.FR_kD));

  private SwerveModule frontLeft =
      new SwerveModule(
          Constants.ids.FL_SPEED,
          Constants.ids.FL_ANGLE,
          Constants.ids.FL_ENCODER,
          Constants.drivetrain.FL_ZERO,
          new PIDController(
              Constants.drivetrain.angle.FL_kP,
              Constants.drivetrain.angle.FL_kI,
              Constants.drivetrain.angle.FL_kD));

  private SwerveModule backLeft =
      new SwerveModule(
          Constants.ids.BL_SPEED,
          Constants.ids.BL_ANGLE,
          Constants.ids.BL_ENCODER,
          Constants.drivetrain.BL_ZERO,
          new PIDController(
              Constants.drivetrain.angle.BL_kP,
              Constants.drivetrain.angle.BL_kI,
              Constants.drivetrain.angle.BL_kD));

  private SwerveModule backRight =
      new SwerveModule(
          Constants.ids.BR_SPEED,
          Constants.ids.BR_ANGLE,
          Constants.ids.BR_ENCODER,
          Constants.drivetrain.BR_ZERO,
          new PIDController(
              Constants.drivetrain.angle.BR_kP,
              Constants.drivetrain.angle.BR_kI,
              Constants.drivetrain.angle.BR_kD));
  private Sensors sensors = Sensors.getInstance();

  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          Constants.drivetrain.FL_LOCATION,
          Constants.drivetrain.FR_LOCATION,
          Constants.drivetrain.BL_LOCATION,
          Constants.drivetrain.BR_LOCATION);

  private SwerveDrivePoseEstimator odometry =
      new SwerveDrivePoseEstimator(
          kinematics,
          sensors.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getModulePosition(),
            frontRight.getModulePosition(),
            backLeft.getModulePosition(),
            backRight.getModulePosition()
          },
          new Pose2d());

  private PIDController rotPID =
      new PIDController(
          Constants.drivetrain.correctionPID.kP, 0.0, Constants.drivetrain.correctionPID.kD);

  private Drivetrain() {
    putDashboard();
    rotPID.enableContinuousInput(-180, 180);
    rotPID.setTolerance(0.5);
  }

  @Override
  public void periodic() {
    updateOdometry();
    SmartDashboard.putString("Position", getPose2d().toString());

    SmartDashboard.putNumber("FL Angle", frontLeft.getModulePosition().angle.getDegrees());
    SmartDashboard.putNumber("BL Angle", backLeft.getModulePosition().angle.getDegrees());
    SmartDashboard.putNumber("BR Angle", backRight.getModulePosition().angle.getDegrees());
    SmartDashboard.putNumber("FR Angle", frontRight.getModulePosition().angle.getDegrees());
  }

  public void putDashboard() {

  }

  public void drive(double x, double y, double rot) {
    double totalSpeed = Math.pow(Math.hypot(x, y), 3.0);
    double angle = Math.atan2(y, x);
    double xSpeed = totalSpeed * Math.cos(angle) * Constants.drivetrain.MAX_VELOCITY;
    double ySpeed = totalSpeed * Math.sin(angle) * Constants.drivetrain.MAX_VELOCITY;
    double rotSpeed = -rot * Constants.drivetrain.MAX_RADIANS;

    if (rotSpeed != 0.0) {
      rotPID.setSetpoint(-sensor.getRotation2d().getDegrees());
    } else if (ySpeed != 0 || xSpeed != 0) {
      double correction = rotPID.calculate(-sensor.getRotation2d().getDegrees());
      rotSpeed = rotPID.atSetpoint() ? 0.0 : correction;
    }

    boolean noInput = xSpeed == 0 && ySpeed == 0 && rotSpeed == 0;

    SwerveModuleState[] states =
        noInput
            ? new SwerveModuleState[] {
              new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
              new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
              new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
              new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            }
            : kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotSpeed, sensors.getRotation2d()));

    setModuleStates(states);
  }

  public void setRotSetpoint(double setpoint) {
    rotPID.setSetpoint(setpoint);
  }

  public void setRotPID(double kP, double kI, double kD) {
    rotPID.setPID(kP, kI, kD);
  }

  public void setRotation(double rotation) {
    rotPID.calculate(sensors.getRotation2d().getDegrees(), rotation);
  }

  public void updateOdometry() {
    Pose2d pose = sensors.getPose2d();
    if (!pose.equals(new Pose2d())
        && pose.getTranslation().getDistance(getPose2d().getTranslation()) < 1.0) {
      // odometry.addVisionMeasurement(pose, sensors.getLatency());
    }

    odometry.update(
        sensors.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getModulePosition(),
          frontRight.getModulePosition(),
          backLeft.getModulePosition(),
          backRight.getModulePosition()
        });
  }

  public void resetOdometry(Pose2d initPose) {
    odometry.resetPosition(
        sensors.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getModulePosition(),
          frontRight.getModulePosition(),
          backLeft.getModulePosition(),
          backRight.getModulePosition()
        },
        initPose);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.drivetrain.MAX_VELOCITY);

    frontLeft.setModuleState(states[0]);
    frontRight.setModuleState(states[1]);
    backLeft.setModuleState(states[2]);
    backRight.setModuleState(states[3]);
  }

  public void zeroPower() {
    frontRight.zeroPower();
    frontLeft.zeroPower();
    backLeft.zeroPower();
    backRight.zeroPower();
  }

  public void setVelocity(double velocity) {
    frontRight.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
    frontLeft.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
    backLeft.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
    backRight.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
  }

  public void setAngle(double angle) {
    frontRight.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
    frontLeft.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
    backLeft.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
    backRight.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
  }

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] states =
        new SwerveModuleState[] {
          frontLeft.getModuleState(),
          frontRight.getModuleState(),
          backLeft.getModuleState(),
          backRight.getModuleState()
        };

    return kinematics.toChassisSpeeds(states);
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose2d() {
    return odometry.getEstimatedPosition();
  }

  public PIDConstants getTransValues() {
    return new PIDConstants(
        Constants.drivetrain.xyPID.kP,
        Constants.drivetrain.xyPID.kI,
        Constants.drivetrain.xyPID.kD);
  }

  public PIDConstants getRotValues() {
    return new PIDConstants(
        Constants.drivetrain.rotPID.kP,
        Constants.drivetrain.rotPID.kI,
        Constants.drivetrain.rotPID.kD);
  }

  public double getSpeed() {
    ChassisSpeeds speeds = getChassisSpeeds();

    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public double getAngularVelocity() {
    return getChassisSpeeds().omegaRadiansPerSecond;
  }
}
