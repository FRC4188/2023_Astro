// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import csplib.motors.CSP_Falcon;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {
    private CSP_Falcon speed;
    private CSP_Falcon angle; 
    private WPI_CANCoder encoder;
    private double zero;
    private double anglekP;
    private double anglekI;
    private double anglekD;

    public SwerveModule(int speedID, int angleID, int encoderID, double zero, double anglekP, double anglekI, double anglekD) {
        speed = new CSP_Falcon(speedID);
        angle = new CSP_Falcon(angleID);
        encoder = new WPI_CANCoder(encoderID);
        this.zero = zero;
        this.anglekP = anglekP;
        this.anglekI = anglekI;
        this.anglekD = anglekD;

        init();
    }

    private void init() {
        speed.setBrake(true);
        speed.setRampRate(Constants.drivetrain.RAMP_RATE);
        speed.setPIDF(Constants.drivetrain.speed.kP, Constants.drivetrain.speed.kI, Constants.drivetrain.speed.kD, 0);

        angle.configFactoryDefault();
        angle.setBrake(true);
        angle.setScalar(Constants.drivetrain.ANGLE_DEGREES_PER_TICK);  
        angle.setEncoder(signedToUnsigned(encoder.getAbsolutePosition() - zero));
        angle.configFeedbackNotContinuous(true, 0);
        angle.setPIDF(anglekP, anglekI, anglekD, anglekD);

        encoder.configFactoryDefault();
        encoder.clearStickyFaults();
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.configSensorDirection(false);
        encoder.configMagnetOffset(-zero);
    }

    public void setModuleState(SwerveModuleState desired) {
        SwerveModuleState optimized = SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(getAngle()));
        speed.setVelocity(optimized.speedMetersPerSecond);
        angle.setPosition(optimized.angle.getDegrees());
    }

    public void setSpeedPIDF(double kP, double kI, double kD, double kF) {
        speed.setPIDF(kP, kI, kD, kF);
    }
    
    public void setAnglePIDF(double kP, double kI, double kD, double kF) {
        angle.setPIDF(kP, kI, kD, kF);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(speed.getVelocity(), Rotation2d.fromDegrees(getAngle()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(speed.getPosition(), Rotation2d.fromDegrees(getAngle()));
    }

    private double getAngle() {
        double angleAngle = signedToUnsigned(angle.getPosition());
        if (Math.abs(angleAngle - encoder.getAbsolutePosition()) > 1) {
            angle.setEncoder(signedToUnsigned(encoder.getAbsolutePosition() - zero));
        }

        return angleAngle;
    }

    private double signedToUnsigned(double input) {
        return (-input + 360.0) % 360.0 - 180.0;
    }

    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition();
    }

    public double[] getTemperature() {
        return new double[] {speed.getTemperature(), angle.getTemperature()};
    }

    public void zeroPower() {
        angle.set(0.0);
        speed.set(0.0);
    }

}

