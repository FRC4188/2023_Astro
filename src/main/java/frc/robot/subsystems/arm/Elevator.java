// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import csplib.motors.CSP_SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static Elevator instance = null;

  ProfiledPIDController elevatorPID = new ProfiledPIDController(0, 0, 0, null);
  SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);


  public static synchronized Elevator getInstance() {
    if (instance == null) instance = new Elevator();
    return instance;
  }

  public enum elevatorHeight {
    GROUND(0),
    MIDDLE(1),
    TOP(2);

    private int height;
    private elevatorHeight(int height) {
      this.height = height;
    }

    public int getLevel() {
      return height;
    }
  }

  public enum elevatorMode {
    RETRACT,
    EXTEND;
  }

  public elevatorMode elevatorMode;
  public elevatorHeight elevatorHeight;

  private CSP_SparkMax motor = new CSP_SparkMax(0); // no clue what the id is yet
  

  public Elevator() {


    CommandScheduler.getInstance().registerSubsystem(this);

    SmartDashboard.putNumber("Elevator Set Voltage", 0.0);
  //  setBrake(true);
  }

  //Sets the elevator mode to "retract" 
  public void resetElevator(){
    elevatorMode = elevatorMode.RETRACT;
  }



  public void setAngleWithFF(double goal){
    motor.setVoltage(elevatorPID.calculate(getElevatorPosition(), goal)+ff.calculate(getElevatorPosition()));
  }

  public void setPosition(double position){
  
  }
  
  public double getElevatorPosition(){
    return motor.getPosition();
  }

  //Updates ShuffleBoard with information about the elevator
  private void updateShuffleboard() {
    // SmartDashboard.putNumber("Elevator Height", getElevatorHeight)); //doesn't work for now
    // SmartDashboard.putString("Elevator Mode", getElevatorMode()); //doen't work for now
    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());

  }

  //Returns elevator mode for shuttleboard
  // public elevatorMode getElevatorMode()  
  // {
  //   return elevatorMode;
  // }

  //Returns elevator height for shuttleboard
  // public elevatorHeight getElevatorHeight() 
  // {
  //   return elevatorHeight;
  // }






  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
