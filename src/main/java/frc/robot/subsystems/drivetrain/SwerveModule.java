// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import csplib.motors.CSP_Falcon;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {
    private CSP_Falcon speed;
    private CSP_Falcon angle; 
    private CANCoder encoder;
    private double zero;

    public SwerveModule(int speedID, int angleID, int encoderID, double zero) {
        speed = new CSP_Falcon(speedID);
        angle = new CSP_Falcon(angleID);
        encoder = new CANCoder(encoderID);
        this.zero = zero;
    

    }

    private void init() {
        speed.init();
        speed.setBrake(true);
        speed.setRampRate(Constants.drivetrain.RAMP_RATE);
        speed.setPIDF(Constants.drivetrain.speedmotor.kP, Constants.drivetrain.speedmotor.kI,Constants.drivetrain.speedmotor.kD, 0);

        angle.init();
        angle.setBrake(true);
        angle.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        encoder.clearStickyFaults();
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.setPosition(0.0);
        encoder.configSensorDirection(false);
        encoder.configMagnetOffset(zero);
    }

    public void setModuleState(SwerveModuleState desired) {
        SwerveModuleState optimized = SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(encoder.getAbsolutePosition()));
        speed.setVelocity(optimized.speedMetersPerSecond);
        angle.setPosition(optimized.angle.getDegrees());

    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(speed.getVelocity(), Rotation2d.fromDegrees(encoder.getAbsolutePosition()));
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(speed.getPosition(), Rotation2d.fromDegrees(encoder.getAbsolutePosition()));

         
    }

    public double[] getTemperature() {
        return new double[] {speed.getTemperature(),angle.getTemperature()};
    }
}

