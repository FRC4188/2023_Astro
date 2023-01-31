<<<<<<< HEAD
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
=======
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
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
import frc.robot.subsystems.sensors.Sensors;

public class Drivetrain extends SubsystemBase {

<<<<<<< HEAD
  private static Drivetrain instance = null;

  public static synchronized Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();

    return instance;
  }

  private Module leftFront = new Module(1, 2, 21, Constants.drivetrain.modules.M1_ZERO);
  private Module rightFront = new Module(3, 4, 22, Constants.drivetrain.modules.M2_ZERO);
  private Module leftRear = new Module(5, 6, 23, Constants.drivetrain.modules.M3_ZERO);
  private Module rightRear = new Module(7, 8, 24, Constants.drivetrain.modules.M4_ZERO);

  private Sensors sensors = Sensors.getInstance();

  //private Kinematics kinematics = Constants.drive.KINEMATICS;
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    Constants.drivetrain.FrontLeftLocation, 
    Constants.drivetrain.FrontRightLocation, 
    Constants.drivetrain.BackLeftLocation, 
    Constants.drivetrain.BackRightLocation);

  private PIDController rotationPID = new PIDController(0.2, 0.0, 0.01);

  //private Odometry odometry = Odometry.getInstance();
  private Odometry odometry = new Odometry(new Pose2d());

  private Notifier dashboard = new Notifier(() -> smartDashboard());

  /** Creates a new Swerve. */
  private Drivetrain() {
    CommandScheduler.getInstance().registerSubsystem(this);
    rotationPID.enableContinuousInput(-180, 180);
    rotationPID.setTolerance(1.0);

    dashboard.startPeriodic(0.1);
=======
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
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
  }

  @Override
  public void periodic() {
<<<<<<< HEAD
    odometry.update(getChassisSpeeds(), sensors.getRotation());
  }

  private void smartDashboard() {
    SmartDashboard.putNumber("M1 (LF) Angle", leftFront.getAbsoluteAngle());
    SmartDashboard.putNumber("M2 (RF) Angle", rightFront.getAbsoluteAngle());
    SmartDashboard.putNumber("M3 (LR) Angle", leftRear.getAbsoluteAngle());
    SmartDashboard.putNumber("M4 (RR) Angle", rightRear.getAbsoluteAngle());
    SmartDashboard.putString("Chassis Speeds", getChassisSpeeds().toString());
    SmartDashboard.putString("Odometry", odometry.getPose().toString());

    SmartDashboard.putNumber("Falcon 1 Temp", leftFront.getAngleTemp());
    SmartDashboard.putNumber("Falcon 2 Temp", leftFront.getSpeedTemp());
    SmartDashboard.putNumber("Falcon 3 Temp", rightFront.getAngleTemp());
    SmartDashboard.putNumber("Falcon 4 Temp", rightFront.getSpeedTemp());
    SmartDashboard.putNumber("Falcon 5 Temp", leftRear.getAngleTemp());
    SmartDashboard.putNumber("Falcon 6 Temp", leftRear.getSpeedTemp());
    SmartDashboard.putNumber("Falcon 7 Temp", rightRear.getAngleTemp());
    SmartDashboard.putNumber("Falcon 8 Temp", rightRear.getSpeedTemp());
  }

  public void drive(double yInput, double xInput, double rotInput, boolean fieldOriented) {
    yInput *= Constants.drivetrain.MAX_VELOCITY;
    xInput *= -Constants.drivetrain.MAX_VELOCITY;
    rotInput *= 4.0 * Math.PI;

    if (rotInput != 0.0) {
      setRotSetpoint(-sensors.getRotation().getDegrees());
    } else {
      if (yInput != 0 || xInput != 0) {
        double correction = rotationPID.calculate(-sensors.getRotation().getDegrees());// * dropoff;
        rotInput = rotationPID.atSetpoint() ? 0.0 : correction;
      }
    }

    setChassisSpeeds(!fieldOriented ?
    ChassisSpeeds.fromFieldRelativeSpeeds(yInput, xInput, rotInput, sensors.getRotation()) :
    new ChassisSpeeds(yInput, xInput, rotInput));
  }

  public void setRotSetpoint(double setpoint) {
    rotationPID.setSetpoint(setpoint);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    ChassisSpeeds newSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    setModuleStates(kinematics.toSwerveModuleStates(newSpeeds));
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 4.75);
    
    leftFront.setModuleState(new SwerveModuleState(states[0].speedMetersPerSecond, states[0].angle));
    rightFront.setModuleState(new SwerveModuleState(states[1].speedMetersPerSecond, states[1].angle));
    leftRear.setModuleState(new SwerveModuleState(states[2].speedMetersPerSecond, states[2].angle));
    rightRear.setModuleState(new SwerveModuleState(states[3].speedMetersPerSecond, states[3].angle));
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

  public void setPose(Pose2d pose) {
    odometry.setPose(pose);
  }

  public Pose2d getPose() {
    return odometry.getPose();
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  
  
}
=======
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
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
