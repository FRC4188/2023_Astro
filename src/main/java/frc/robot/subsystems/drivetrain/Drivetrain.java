package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.sensors.Sensors;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance;
  public static synchronized Drivetrain getInstance(){
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  private SwerveModule frontRight = new SwerveModule(Constants.ids.FR_SPEED, Constants.ids.FR_ANGLE, 
                                                    Constants.ids.FR_ENCODER, Constants.drivetrain.M1_ZERO, 
                                                    new ProfiledPIDController(Constants.drivetrain.angle.M1_kP, 
                                                    Constants.drivetrain.angle.M1_kI, Constants.drivetrain.angle.M1_kD,
                                                    new Constraints(Constants.drivetrain.angle.MAX_OMEGA, Constants.drivetrain.angle.MAX_ALPHA)),
                                                    new SimpleMotorFeedforward(Constants.drivetrain.angle.M1_kS, Constants.drivetrain.angle.M1_kV));

  private SwerveModule frontLeft = new SwerveModule(Constants.ids.FL_SPEED, Constants.ids.FL_ANGLE, 
                                                     Constants.ids.FL_ENCODER, Constants.drivetrain.M2_ZERO, 
                                                     new ProfiledPIDController( Constants.drivetrain.angle.M2_kP, 
                                                     Constants.drivetrain.angle.M2_kI, Constants.drivetrain.angle.M2_kD,
                                                     new Constraints(Constants.drivetrain.angle.MAX_OMEGA, Constants.drivetrain.angle.MAX_ALPHA)),
                                                     new SimpleMotorFeedforward(Constants.drivetrain.angle.M2_kS, Constants.drivetrain.angle.M2_kV));

  private SwerveModule backLeft = new SwerveModule(Constants.ids.BL_SPEED, Constants.ids.BL_ANGLE, 
                                                     Constants.ids.BL_ENCODER, Constants.drivetrain.M3_ZERO, 
                                                     new ProfiledPIDController(Constants.drivetrain.angle.M3_kP, 
                                                     Constants.drivetrain.angle.M3_kI, Constants.drivetrain.angle.M3_kD,
                                                     new Constraints(Constants.drivetrain.angle.MAX_OMEGA, Constants.drivetrain.angle.MAX_ALPHA)),
                                                     new SimpleMotorFeedforward(Constants.drivetrain.angle.M2_kS, Constants.drivetrain.angle.M2_kV));

  private SwerveModule backRight = new SwerveModule(Constants.ids.BL_SPEED, Constants.ids.BL_ANGLE, 
                                                    Constants.ids.BR_ENCODER, Constants.drivetrain.M4_ZERO, 
                                                    new ProfiledPIDController( Constants.drivetrain.angle.M3_kP, 
                                                    Constants.drivetrain.angle.M3_kI, Constants.drivetrain.angle.M3_kD,
                                                    new Constraints(Constants.drivetrain.angle.MAX_OMEGA, Constants.drivetrain.angle.MAX_ALPHA)),
                                                    new SimpleMotorFeedforward(Constants.drivetrain.angle.M3_kS, Constants.drivetrain.angle.M3_kV));
                                                    private Sensors sensors = Sensors.getInstance();

  private int moduleNum = 0;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    Constants.drivetrain.FL_LOCATION, 
    Constants.drivetrain.FR_LOCATION, 
    Constants.drivetrain.BL_LOCATION, 
    Constants.drivetrain.BR_LOCATION);

  // private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
  //   kinematics, 
  //   pigeon.getAngle(), 
  //   new SwerveModulePosition[] {
  //     frontLeft.getModulePosition(), 
  //     frontRight.getModulePosition(), 
  //     backLeft.getModulePosition(), 
  //     backRight.getModulePosition()});

  private SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
    kinematics, 
    sensors.getPigeonAngle(), 
    new SwerveModulePosition[] {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition()
    }, 
    new Pose2d(), 
    Constants.drivetrain.STATE_STD_DEVS, 
    Constants.drivetrain.VISION_STD_DEVS);

  private Drivetrain() {
    putDashboard();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("M1 Angle", frontRight.getModulePosition().angle.getDegrees());
    SmartDashboard.putNumber("M2 Angle", frontLeft.getModulePosition().angle.getDegrees());
    SmartDashboard.putNumber("M3 Angle", backLeft.getModulePosition().angle.getDegrees());
    SmartDashboard.putNumber("M4 Angle", backRight.getModulePosition().angle.getDegrees());
  }

  public void putDashboard() {
    SmartDashboard.putNumber("Set Drive Velocity", 0);
    SmartDashboard.putNumber("Speed kP", 0);
    SmartDashboard.putNumber("Speed kI", 0);
    SmartDashboard.putNumber("Speed kD", 0);
    SmartDashboard.putNumber("Speed kF", 0);

    SmartDashboard.putNumber("Set Drive Angle", 0);
    SmartDashboard.putNumber("Angle kP", 0);
    SmartDashboard.putNumber("Angle kI", 0);
    SmartDashboard.putNumber("Angle kD", 0);
    SmartDashboard.putNumber("Angle kF", 0);

    SmartDashboard.putNumber("Set Module", 0);

  }

  public void setVelocity(double velocity) {
    switch (moduleNum) {
      case 1:
        frontRight.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
        break;
      case 2:
        frontLeft.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
        break;
      case 3:
        backLeft.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
        break;
      case 4:
        backRight.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
      default:
        break;
    }
  }

  public void setAngle(double angle) {
    switch (moduleNum) {
      case 1:
        frontRight.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
        break;
      case 2:
        frontLeft.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
        break;
      case 3:
        backLeft.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
        break;
      case 4:
        backRight.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
      default:
        break;
    }
  }

  public void setSpeedPIDs(double kP, double kI, double kD, double kF) {
    switch (moduleNum) {
      case 1:
        frontRight.setSpeedPIDF(kP, kI, kD, kF);
        break;
      case 2:
        frontLeft.setSpeedPIDF(kP, kI, kD, kF);
        break;
      case 3:
        backLeft.setSpeedPIDF(kP, kI, kD, kF);
        break;
      case 4:
        backRight.setSpeedPIDF(kP, kI, kD, kF);
      default:
        break;
    }
  }

  public void setAnglePIDs(double kP, double kI, double kD, double kF) {
    switch (moduleNum) {
      case 1:
        frontRight.setAnglePID(kP, kI, kD);
        break;
      case 2:
        frontLeft.setAnglePID(kP, kI, kD);
        break;
      case 3:
        backLeft.setAnglePID(kP, kI, kD);
        break;
      case 4:
        backRight.setAnglePID(kP, kI, kD);
      default:
        break;
    }
  }

  public void zeroPower() {
    frontRight.zeroPower();
    frontLeft.zeroPower();
    backLeft.zeroPower();
    backRight.zeroPower();
  }

  public void setModuleNum(int moduleNum) {
    if (moduleNum >= 1 && moduleNum <= 4) {
      this.moduleNum = moduleNum;
    }
    System.out.println(this.moduleNum);
  }
}