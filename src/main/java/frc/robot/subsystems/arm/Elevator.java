// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import csplib.motors.CSP_Motor;
import csplib.motors.CSP_SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static Elevator Elevator;

  public static synchronized Elevator getInstance()
  {
    if (Elevator == null)
    {
      Elevator = new Elevator();
    }
    return Elevator;
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

  private CSP_Motor motor = new CSP_SparkMax(0);

  public Elevator() {

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  //Sets the elevator mode to "retract" 
  public void resetElevator()
  {
    elevatorMode = elevatorMode.RETRACT;
  }

  //Updates ShuffleBoard with information about the elevator
  private void updateShuffleboard() {
    SmartDashboard.putNumber("Elevator Height", getElevatorHeight)); //doesn't work for now
    SmartDashboard.putString("Elevator Mode", getElevatorMode()); //doesn't work for now
  }

  //Returns elevator mode for shuttleboard
  public elevatorMode getElevatorMode()  
  {
    return elevatorMode;
  }

  //Returns elevator height for shuttleboard
  public elevatorHeight getElevatorHeight() 
  {
    return elevatorHeight;
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
