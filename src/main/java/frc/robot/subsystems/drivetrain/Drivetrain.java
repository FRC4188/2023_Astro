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

  private SwerveModule frontRight = new SwerveModule(Constants.ids.FR_SPEED, Constants.ids.FR_ANGLE, 
                                                    Constants.ids.FR_ENCODER, Constants.drivetrain.M1_ZERO, 
                                                    Constants.drivetrain.angle.M1_kP, Constants.drivetrain.angle.M1_kI, 
                                                    Constants.drivetrain.angle.M1_kD);

  private SwerveModule frontLeft = new SwerveModule(Constants.ids.FL_SPEED, Constants.ids.FL_ANGLE, 
                                                    Constants.ids.FL_ENCODER, Constants.drivetrain.M2_ZERO,
                                                    Constants.drivetrain.angle.M2_kP, Constants.drivetrain.angle.M2_kI, Constants.drivetrain.angle.M2_kD);

  private SwerveModule backLeft = new SwerveModule(Constants.ids.BL_SPEED, Constants.ids.BL_ANGLE, 
                                                  Constants.ids.BL_ENCODER, Constants.drivetrain.M3_ZERO,
                                                  Constants.drivetrain.angle.M3_kP, Constants.drivetrain.angle.M3_kI, Constants.drivetrain.angle.M3_kD);

  private SwerveModule backRight = new SwerveModule(Constants.ids.BR_SPEED, Constants.ids.BR_ANGLE, 
                                                    Constants.ids.BR_ENCODER, Constants.drivetrain.M4_ZERO,
                                                    Constants.drivetrain.angle.M4_kP, Constants.drivetrain.angle.M4_kI, Constants.drivetrain.angle.M4_kD);

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

    SmartDashboard.putNumber("M1 Angle", frontRight.getModuleState().angle.getDegrees());
    SmartDashboard.putString("Angle PID", frontRight.getPID().getP() + ", " + frontRight.getPID().getI() + ", " + frontRight.getPID().getD());
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

  }

  public void setVelocity(double velocity) {
    frontLeft.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
    frontRight.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
    backLeft.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
    backRight.setModuleState(new SwerveModuleState(velocity, new Rotation2d()));
  }

  public void setAngle(double angle) {
    frontRight.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));

  }

  public void setSpeedPIDs(double kP, double kI, double kD, double kF) {
    frontLeft.setSpeedPIDF(kP, kI, kD, kF);
    frontRight.setSpeedPIDF(kP, kI, kD, kF);
    backLeft.setSpeedPIDF(kP, kI, kD, kF);
    backRight.setSpeedPIDF(kP, kI, kD, kF);
  }

  public void setAnglePIDs(double kP, double kI, double kD, double kF) {
    frontRight.setAnglePIDF(kP, kI, kD, kF);
  }

  public void zeroPower() {
    frontRight.zeroPower();
  }

}