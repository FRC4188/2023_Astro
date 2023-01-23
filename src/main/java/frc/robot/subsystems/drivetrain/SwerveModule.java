// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

    private PIDController anglePID;

    public SwerveModule(int speedID, int angleID, int encoderID, double zero, double anglekP, double anglekI, double anglekD) {
        speed = new CSP_Falcon(speedID);
        angle = new CSP_Falcon(angleID);
        encoder = new WPI_CANCoder(encoderID);
        this.zero = zero;

        anglePID = new PIDController(anglekP, anglekI, anglekD);

        init();
    }

    private void init() {
        speed.setBrake(true);
        speed.setRampRate(Constants.drivetrain.RAMP_RATE);
        speed.setPIDF(Constants.drivetrain.speed.kP, Constants.drivetrain.speed.kI, Constants.drivetrain.speed.kD, 0);
        speed.setScalar(Constants.drivetrain.DRIVE_COUNTS_PER_METER);

        angle.setBrake(true);
        angle.setEncoder(0.0);
        anglePID.enableContinuousInput(-180, 180);

        encoder.configFactoryDefault();
        encoder.clearStickyFaults();
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.setPosition(0.0);
        encoder.configSensorDirection(false);
        encoder.configMagnetOffset(-zero);
        }

    public void setModuleState(SwerveModuleState desired) {
        SwerveModuleState optimized = SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(encoder.getAbsolutePosition()));
        speed.setVelocity(optimized.speedMetersPerSecond);
        angle.set(anglePID.calculate(encoder.getAbsolutePosition(), optimized.angle.getDegrees()));
    }

    public void setSpeedPIDF(double kP, double kI, double kD, double kF) {
        speed.setPIDF(kP, kI, kD, kF);
    }
    
    public void setAnglePIDF(double kP, double kI, double kD, double kF) {
        anglePID.setPID(kP, kI, kD);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(speed.getVelocity(), Rotation2d.fromDegrees(encoder.getAbsolutePosition()));
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(speed.getPosition(), Rotation2d.fromDegrees(encoder.getAbsolutePosition()));
    }

    public double[] getTemperature() {
        return new double[] {speed.getTemperature(), angle.getTemperature()};
    }

    public void zeroPower() {
        angle.set(ControlMode.PercentOutput, 0.0);
        speed.set(ControlMode.PercentOutput, 0.0);
    }

    public PIDController getPID() {
        return anglePID;
    }
}

