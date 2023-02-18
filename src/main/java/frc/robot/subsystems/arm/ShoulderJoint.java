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
public class ShoulderJoint {

    private CSP_SparkMax leader = new CSP_SparkMax(Constants.ids.SHOULDER_LEADER);

    private WPI_CANCoder encoder = new WPI_CANCoder(Constants.ids.SHOULDER_ENCODER);
    
    private ProfiledPIDController pidder = new ProfiledPIDController(0, 0, 0, null);

    private ArmFeedforward ff = new ArmFeedforward(0, 0, 0);

    public ShoulderJoint() {
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
        leader.setPIDF(Constants.arm.shoulder.kP, Constants.arm.shoulder.kI, Constants.arm.shoulder.kD, Constants.arm.shoulder.kF);

    }

    public void setPosition(double position) {
        leader.setPosition(position);
    }

    public double getAngle(){
        return(leader.getPosition()*Constants.arm.shoulder.TICKS_PER_DEGREE);
    }
    
    

    public void setAngleWithPID(double goal){
        leader.setVoltage(pidder.calculate(getAngle(), goal)+ff.calculate(pidder.getSetpoint().velocity, 0));
    }


    
      


}