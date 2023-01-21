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

public class Wrist extends SubsystemBase {

  private static Wrist Wrist;
  private final ArmFeedforward m_feedforward =
  new ArmFeedforward(
    Constants.wrist.kS, Constants.wrist.kG, Constants.wrist.kV, Constants.wrist.kA);
    private static ProfiledPIDController pid = new ProfiledPIDController(
        Constants.wrist.Kp, Constants.wrist.Ki, Constants.wrist.Kd, Constants.wrist.constriants, 0.0);

  public static synchronized Wrist getInstance()
  {
    if (Wrist == null)
    {
      Wrist = new Wrist();
    }
    return Wrist;
  }

  private CSP_Motor motor = new CSP_SparkMax(0); // no clue what the id is yet

  public Wrist() {

    CommandScheduler.getInstance().registerSubsystem(this);

  }

  //Updates ShuffleBoard with information about the Wrist
  private void updateShuffleboard() {

    SmartDashboard.putNumber("Temperature", getTemperature());
    SmartDashboard.putNumber("Position", getPosition());
    SmartDashboard.putNumber("Angle", getAngle());
   
  }

  public double getTemperature()
  {
    return motor.getTemperature();
  }

  public double getPosition()
  {
    return motor.getPosition();
  }

  public double getAngle() // WIP (prob doesn't work)
  {
    return ((getPosition() / 48.0) / 120.0) * 360.0 + 30.0;
  }

  public void set(double power)
  {
    motor.set(power);
  }

  public void setAngle(double angle)
  {
    set(pid.calculate(getAngle(), angle));
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() { 

    updateShuffleboard();

  } 

}
