// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import csplib.motors.CSP_SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static Elevator instance = null;

  ProfiledPIDController elevatorPID = new ProfiledPIDController(0, 0, 0, null);
  SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);

  private WPI_CANCoder encoder = new WPI_CANCoder(Constants.ids.SHOULDER_ENCODER);

  public static synchronized Elevator getInstance() {
    if (instance == null) instance = new Elevator();
    return instance;
  }

  private void init() {
    encoder.configFactoryDefault();
    encoder.clearStickyFaults();
    encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    encoder.setPosition(0.0);
    encoder.configSensorDirection(false);
    encoder.configMagnetOffset(-Constants.arm.shoulder.ZERO);

    motor.setScalar(1 / Constants.arm.shoulder.TICKS_PER_DEGREE);
    motor.setBrake(true);
    motor.setPosition(encoder.getAbsolutePosition());
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.arm.shoulder.UPPER_LIMIT);
    motor.setPIDF(Constants.arm.shoulder.kP, Constants.arm.shoulder.kI, Constants.arm.shoulder.kD, Constants.arm.shoulder.kF);

}

  private CSP_SparkMax motor = new CSP_SparkMax(0); // no clue what the id is yet

  DigitalInput limitSwitch = new DigitalInput(0);

  
  

  public Elevator() {
    CommandScheduler.getInstance().registerSubsystem(this);
    init();

    SmartDashboard.putNumber("Elevator Set Voltage", 0.0);
    }

  public void setAngleWithFF(double goal){
    motor.setVoltage(elevatorPID.calculate(getElevatorPosition(), goal)+ff.calculate(elevatorPID.getSetpoint().velocity, 0.0));
  }

  public double getElevatorPosition(){
    return motor.getPosition()*Constants.arm.elevator.TICKS_PER_METER;
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
