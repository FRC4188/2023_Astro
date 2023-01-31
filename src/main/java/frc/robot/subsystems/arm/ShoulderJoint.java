// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import csplib.motors.CSP_Motor;
import csplib.motors.CSP_SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShoulderJoint extends SubsystemBase { //uses two motors

  private static ShoulderJoint ShoulderJoint;
  // private final ArmFeedforward m_feedforward =
  // new ArmFeedforward(
  //   Constants.ShoulderJoint.kS, Constants.ShoulderJoint.kG, Constants.ShoulderJoint.kV, Constants.ShoulderJoint.kA);
  //   private static ProfiledPIDController pid = new ProfiledPIDController(
  //       Constants.ShoulderJoint.Kp, Constants.ShoulderJoint.Ki, Constants.ShoulderJoint.Kd, Constants.ShoulderJoint.constriants, 0.0);

  public static synchronized ShoulderJoint getInstance()
  {
    if (ShoulderJoint == null)
    {
      ShoulderJoint = new ShoulderJoint();
    }
    return ShoulderJoint;
  }

  private CSP_Motor motor = new CSP_SparkMax(0); // no clue what the id is yet
  private CSP_Motor motorTwo = new CSP_SparkMax(1); // no clue what the id is yet

  public ShoulderJoint() {

    CommandScheduler.getInstance().registerSubsystem(this);

  }

  //Updates ShuffleBoard with information about the ShoulderJoint
  private void updateShuffleboard() {

    SmartDashboard.putNumber("Temperature", getTemperature());
    SmartDashboard.putNumber("Position", getPosition());
    SmartDashboard.putNumber("Angle", getAngle());
;\
  }

  public double getTemperature()
  {
    return motor.getTemperature();
  }

  public double getPosition()
  {
    return motor.getPosition();
  }

  public double getAngle() // (we don't have the correct double values yet)
  {
    return ((getPosition() / 48.0) / 120.0) * 360.0 + 30.0;
  }

  public void set(double power)
  {
    motor.set(power);
  }

  public void setAngle(double angle)
  {
    // set(pid.calculate(getAngle(), angle));
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() { 

    updateShuffleboard();

  } 

}
