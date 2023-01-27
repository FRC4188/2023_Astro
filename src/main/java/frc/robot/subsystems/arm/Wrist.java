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
  // private final ArmFeedforward m_feedforward =
  // new ArmFeedforward(
  //   Constants.wrist.kS, Constants.wrist.kG, Constants.wrist.kV, Constants.wrist.kA);
  //   private static ProfiledPIDController pid = new ProfiledPIDController(
  //       Constants.wrist.Kp, Constants.wrist.Ki, Constants.wrist.Kd, Constants.wrist.constriants, 0.0);

  public static synchronized Wrist getInstance()
  {
    if (Wrist == null)
    {
      Wrist = new Wrist();
    }
    return Wrist;
  }

  private CSP_Motor armMotor = new CSP_SparkMax(0); // no clue what the id is yet
  private CSP_Motor wristMotor = new CSP_SparkMax(1); // no clue what the id is yet

  public Wrist() {

    CommandScheduler.getInstance().registerSubsystem(this);

  }

  //Updates ShuffleBoard with information about the Wrist
  private void updateShuffleboard() {

    SmartDashboard.putNumber("(Wrist) Arm Temperature", getArmTemperature());
    SmartDashboard.putNumber("(Wrist) Wrist Temperature", getArmTemperature());
    SmartDashboard.putNumber("(Wrist) Arm Position", getArmPosition());
    SmartDashboard.putNumber("(Wrist) Wrist Position", getWristPosition());
   
  }

  public double getArmTemperature()
  {
    return armMotor.getTemperature();
  }

  public double getWristTemperature()
  {
    return wristMotor.getTemperature();
  }

  public double getArmPosition()
  {
    return armMotor.getPosition();
  }

  public double getWristPosition()
  {
    return wristMotor.getPosition();
  }

  public double getAngle(CSP_Motor motor) // WIP 
  {
    return 0.0;
  }

  public void set(String motor, double power)
  {
    if (motor == "arm")
    armMotor.set(power);
    else if (motor == "wrist")
    wristMotor.set(power);
    
  }

  public void setAngle(String motor, double angle) // WIP
  {
    // set(pid.calculate(getAngle(), angle));
    set(motor, angle);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() { 

    updateShuffleboard();

  } 

}
