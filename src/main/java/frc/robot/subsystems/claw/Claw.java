package frc.robot.subsystems.claw;

import csplib.motors.CSP_Talon;
import csplib.utils.TempManager;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  private static Claw instance;

  public static synchronized Claw getInstance() {
    if (instance == null) instance = new Claw();
    return instance;
  }

  private CSP_Talon motor = new CSP_Talon(Constants.ids.CLAW);
  private AnalogPotentiometer sensor =
      new AnalogPotentiometer(Constants.ids.ULTRASONIC_SENSOR, Constants.claw.SENSOR_SCALE);

  private Claw() {
    TempManager.addMotor(motor);
  }

  public void set(double percent) {
    motor.set(percent);
  }

  public double getDistance() {
    return sensor.get();
  }
}
