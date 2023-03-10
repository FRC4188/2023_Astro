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
  
  private boolean isCube;

  private Claw() {
    TempManager.addMotor(motor);
  }

  private void init() {
    motor.setBrake(true);
    motor.setInverted(false);
  }

  public void set(double percent) {
    motor.set(percent);
  }

  private void setInverted(boolean inverted) {
    if (inverted) motor.setInverted(false);
    else motor.setInverted(true);
  }

  public void setIsCube(boolean isCube) {
    this.isCube = isCube;
  }

  public void intake() {
    setInverted(isCube);

    motor.set(1.0);
  }

  public void outtake() {
    setInverted(isCube);

    motor.set(-1.0);
  }

  public double getDistance() {
    return sensor.get();
  }

  public double getClawLength() {
    return (isCube) ? Constants.robot.CUBE_DISTANCE : Constants.robot.TOTAL_DISTANCE;
  }

  public boolean getIsCube() {
    return isCube;
  }
}
