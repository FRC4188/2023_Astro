package frc.robot.subsystems.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensors extends SubsystemBase {

  private static Sensors instance = null;

  public static synchronized Sensors getInstance() {
    if (instance == null) instance = new Sensors();
    return instance;
  }

  

  private SendableChooser<String> alliance = new SendableChooser<>();


 
  /** Creates a new Sensors. */
  private Sensors() {
    CommandScheduler.getInstance().registerSubsystem(this);
    
    alliance.setDefaultOption("FMS", "FMS");
    alliance.addOption("Blue", "Blue");
    alliance.addOption("Red", "Red");
    alliance.addOption("All", "All");

    SmartDashboard.putData("Alliance Color", alliance);

    //setPower(true);
  }

  public void updateDashboard() {
  }

  public void setLED(boolean on) {
    //limelight.setLEDMode(on ? LedMode.ON : LedMode.OFF);
  }

  public Rotation2d getRotation() {
    return null;
  }


  }
