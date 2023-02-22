// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.apache.commons.lang3.ObjectUtils.Null;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import csplib.motors.CSP_SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.Constants.controller;

/** Add your docs here. */
public class Shoulder {

    private CSP_SparkMax leader = new CSP_SparkMax(Constants.ids.SHOULDER_LEADER);
    private CSP_SparkMax follower = new CSP_SparkMax(Constants.ids.SHOULDER_LEADER);
    private WPI_CANCoder encoder = new WPI_CANCoder(Constants.ids.SHOULDER_ENCODER);
    
    private ProfiledPIDController pid = new ProfiledPIDController(Constants.arm.shoulder.kP, Constants.arm.shoulder.kI, Constants.arm.shoulder.kD, Constants.arm.shoulder.CONSTRAINTS);
    private ArmFeedforward ff = new ArmFeedforward(Constants.arm.shoulder.kS, Constants.arm.shoulder.kG, Constants.arm.shoulder.kV);

    public Shoulder() {
        init();
    }

    private void init() {
        encoder.configFactoryDefault();
        encoder.clearStickyFaults();
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.setPosition(0.0);
        encoder.configSensorDirection(false);
        encoder.configMagnetOffset(-Constants.arm.shoulder.ZERO);
        
        leader.setScalar(1 / Constants.arm.shoulder.TICKS_PER_DEGREE);
        leader.setBrake(true);
        leader.setPosition(encoder.getAbsolutePosition());
        leader.enableSoftLimit(SoftLimitDirection.kForward, true);
        leader.enableSoftLimit(SoftLimitDirection.kReverse, true);
        leader.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.arm.shoulder.UPPER_LIMIT);
        leader.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.arm.shoulder.LOWER_LIMIT);

        follower.follow(leader);

    }

    public void setAngle(double goal){
        leader.setVoltage(pid.calculate(getAngle(), goal) + ff.calculate(Math.toRadians(getAngle() + Math.PI / 2), pid.getSetpoint().velocity));
    }

    public void setPID(double kP, double kI, double kD){
        pid.setPID(kP, kI, kD);
    }

    public double getAngle(){
        return encoder.getAbsolutePosition();
    }
    

}