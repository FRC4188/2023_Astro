package frc.robot.subsystems.drivetrain;

import java.util.LinkedList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.AidenLib.control.Odometry3D;
import frc.robot.AidenLib.control.PoseEstimator3D;
import frc.robot.AidenLib.math.Data;
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
          Constants.drive.FL_LOCATION,
          Constants.drive.FR_LOCATION,
          Constants.drive.BL_LOCATION,
          Constants.drive.BR_LOCATION);

  private Odometry3D odometry = new Odometry3D();

  private PIDController rotationPID = new PIDController(0.08, 0.0, 0.0075);

  private Derivative accel = new Derivative(0.0);

  private Pose3d pose = new Pose3d();
  //private PoseEstimator3D estimator = new PoseEstimator3D(new Pose3d(), 0.05);

  private Notifier updateShuffle = new Notifier(() -> updateDashboard());

  private Field2d field = new Field2d();

  /** Creates a new Swerve. */
  private Swerve() {
    instantiateModules();

    rotationPID.enableContinuousInput(-180, 180);
    rotationPID.setTolerance(0.75);

    updateShuffle.startPeriodic(0.2);

    SmartDashboard.putData(field);
  }


  @Override
  public void periodic() {
    // Results results = LimelightHelpers.getLatestResults("limelight").targetingResults;
    // int tags = results.targets_Fiducials.length;
    // LinkedList<Data> x = new LinkedList<>();
    // LinkedList<Data> y = new LinkedList<>();
    // LinkedList<Data> z = new LinkedList<>();

    // if (tags > 0) {
    //   x.add(new Data(results.botpose[0] + Units.inchesToMeters(651.25) / 2.0, Math.pow(0.8, -tags)));
    //   y.add(new Data(-results.botpose[1] + Units.inchesToMeters(315.5) / 2.0, Math.pow(0.8, -tags)));
    //   z.add(new Data(results.botpose[2], Math.pow(3.5, -tags)));
    // }

    // pose = estimator.estimate(getChassisSpeeds(), sensors.getRotation(), x, y, z);
    pose = odometry.update(getChassisSpeeds(), sensors.getRotation());
  }

  public void instantiateModules() {
    leftFront = new Module(4, 3, 12, Constants.drive.mod1zero);
    rightFront = new Module(2, 1, 11, Constants.drive.mod2zero);
    leftRear = new Module(6, 5, 13, Constants.drive.mod3zero);
    rightRear = new Module(8, 7, 14, Constants.drive.mod4zero);
  }

  public void updateDashboard() {
    SmartDashboard.putString("Pose", poseToString(pose));
    // SwerveModuleState[] states = getModuleStates();
    // SmartDashboard.putNumber("Module 1", states[0].angle.getDegrees());
    // SmartDashboard.putNumber("Module 2", states[1].angle.getDegrees());
    // SmartDashboard.putNumber("Module 3", states[2].angle.getDegrees());
    // SmartDashboard.putNumber("Module 4", states[3].angle.getDegrees());

    field.setRobotPose(pose.toPose2d());
  }


  public void drive(double yInput, double xInput, double rotInput) {
    // yInput = yLimiter.calculate(yInput) * Constants.drive.MAX_VELOCITY;
    // xInput = xLimiter.calculate(xInput) * -Constants.drive.MAX_VELOCITY;
    // rotInput = rotLimiter.calculate(rotInput) * 2.0 * Math.PI;
    yInput = yInput * Constants.drive.MAX_VELOCITY;
    xInput = xInput * Constants.drive.MAX_VELOCITY;
    rotInput = rotInput * 2.0 * Math.PI;

    if (xInput != 0.0 || yInput != 0.0 || rotInput != 0.0) {
      if (rotInput != 0.0) {
        setRotSetpoint(-sensors.getRotation().toRotation2d().getDegrees());
      } else if (yInput != 0 || xInput != 0) {
        double correction = rotationPID.calculate(-sensors.getRotation().toRotation2d().getDegrees());
        rotInput = rotationPID.atSetpoint() ? 0.0 : correction;
      }

      double pitch = Math.abs(sensors.getPitch()) < 1.5 ? 0.0 : sensors.getPitch();
      double roll = Math.abs(sensors.getRoll()) < 1.5 ? 0.0 : sensors.getRoll();

      ChassisSpeeds result;

      result = ChassisSpeeds.fromFieldRelativeSpeeds(yInput, xInput, rotInput, sensors.getRotation().toRotation2d());

      setChassisSpeeds(result);
    } else {
      setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0)),
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0))
      });
    }

  }

  public double getStdDevs() {
    double zAccel = sensors.getAccel().rotateBy(sensors.getRotation().times(-1.0)).getZ();
    if (zAccel > -8.0) return 0.01;
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

  public double getRotSetpoint() {
    return rotationPID.getSetpoint();
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    ChassisSpeeds newSpeeds =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    setModuleStates(kinematics.toSwerveModuleStates(newSpeeds));
  }

  public Pose3d getPose() {
    return pose;
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
  private static String poseToString(Pose3d pose) {
    Rotation3d rot = pose.getRotation();
    return String.format("X: %.2f; Y: %.2f; Z: %.2f; Roll: %.2f; Pitch: %.2f; Yaw: %.2f; ", pose.getX(), pose.getY(), pose.getZ(), rot.getX(), rot.getY(), rot.getZ());
  }

  public void setPose(Pose3d pose) {
    this.pose = pose;
    odometry.setPose(pose);
  }

}
