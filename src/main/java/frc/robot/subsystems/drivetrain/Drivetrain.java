package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.drivetrain;
import frc.robot.subsystems.sensors.Pigeon;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance;
  public static synchronized Drivetrain getInstance(){
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  private SwerveModule frontRight = new SwerveModule(Constants.ids.FR_SPEED, Constants.ids.FR_ANGLE, Constants.ids.FR_ENCODER, 0);
  private SwerveModule frontLeft = new SwerveModule(Constants.ids.FL_SPEED, Constants.ids.FL_ANGLE, Constants.ids.FL_ENCODER, 0);
  private SwerveModule backLeft = new SwerveModule(Constants.ids.BL_SPEED, Constants.ids.BL_ANGLE, Constants.ids.BL_ENCODER, 0);
  private SwerveModule backRight = new SwerveModule(Constants.ids.BR_SPEED, Constants.ids.BR_ANGLE, Constants.ids.BR_ENCODER, 0);

  private Pigeon pigeon = new Pigeon(Constants.ids.PIGEON);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    Constants.drivetrain.FL_LOCATION, 
    Constants.drivetrain.FR_LOCATION, 
    Constants.drivetrain.BL_LOCATION, 
    Constants.drivetrain.BR_LOCATION);

  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    kinematics, 
    pigeon.getAngle(), 
    new SwerveModulePosition[] {
      frontLeft.getModulePosition(), 
      frontRight.getModulePosition(), 
      backLeft.getModulePosition(), 
      backRight.getModulePosition()});

  private Drivetrain() {
    putDashboard();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumberArray("Velocities", new double[] {frontLeft.getModuleState().speedMetersPerSecond, frontRight.getModuleState().speedMetersPerSecond, backLeft.getModuleState().speedMetersPerSecond, backRight.getModuleState().speedMetersPerSecond});
    SmartDashboard.putNumberArray("Angles", new double[] {frontLeft.getModuleState().angle.getDegrees(), frontRight.getModuleState().angle.getDegrees(), backLeft.getModuleState().angle.getDegrees(), backRight.getModuleState().angle.getDegrees()});
  }

  public void putDashboard() {
    SmartDashboard.putNumber("Set Drive Velocity", 0);
    SmartDashboard.putNumber("Speed kP", 0);
    SmartDashboard.putNumber("Speed kI", 0);
    SmartDashboard.putNumber("Speed kD", 0);
    SmartDashboard.putNumber("Speed kF", 0);

  }

  public void setVelocity(double velocity) {
    frontLeft.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
    frontRight.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
    backLeft.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
    backRight.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
  }

  public void setSpeedPIDs(double kP, double kI, double kD, double kF) {
    frontLeft.setSpeedPIDF(kP, kI, kD, kF);
    frontRight.setSpeedPIDF(kP, kI, kD, kF);
    backLeft.setSpeedPIDF(kP, kI, kD, kF);
    backRight.setSpeedPIDF(kP, kI, kD, kF);
  }

  public void setAnglePIDs(double kP, double kI, double kD, double kF) {
    frontLeft.setAnglePIDF(kP, kI, kD, kF);
    frontRight.setAnglePIDF(kP, kI, kD, kF);
    backLeft.setAnglePIDF(kP, kI, kD, kF);
    backRight.setAnglePIDF(kP, kI, kD, kF);
  }

}