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

public class Telescope extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static Telescope telescope;

  public static synchronized Telescope getInstance()
  {
    if (telescope == null)
    {
      telescope = new Telescope();
    }
    return telescope;
  }

  public enum telescopeHeight {
    GROUND(0),
    MIDDLE(1),
    TOP(2);

    private int height;
    private telescopeHeight(int height) {
      this.height = height;
    }

    public int getLevel() {
      return height;
    }
  }

  public enum telescopeMode {
    RETRACT,
    EXTEND;
  }

  public telescopeMode telescopeMode;
  public telescopeHeight telescopeHeight;

  private CSP_Motor motor = new CSP_SparkMax(0); // no clue what the id is yet

  public Telescope() {

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  //Sets the telescope mode to "retract" 
  public void resetTelescope()
  {
    telescopeMode = telescopeMode.RETRACT;
  }

  //Updates ShuffleBoard with information about the telescope
  // private void updateShuffleboard() {
  //   SmartDashboard.putNumber("telescope Height", gettelescopeHeight)); //doesn't work for now
  //   SmartDashboard.putString("telescope Mode", gettelescopeMode()); //doesn't work for now
  // }

  //Returns telescope mode for shuttleboard
  // public telescopeMode gettelescopeMode()  
  // {
  //   return telescopeMode;
  // }

  //Returns telescope height for shuttleboard
  // public telescopeHeight gettelescopeHeight() 
  // {
  //   return telescopeHeight;
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
