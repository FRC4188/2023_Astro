package frc.robot.subsystems.claw;

import csplib.motors.CSP_Talon;
import csplib.utils.TempManager;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.arm.wrist;
import frc.robot.subsystems.arm.Shoulder;
import frc.robot.subsystems.arm.Wrist;

public class Claw extends SubsystemBase {
  private static Claw instance;

  public static synchronized Claw getInstance() {
    if (instance == null) instance = new Claw();
    return instance;
  }

  private CSP_Talon motor = new CSP_Talon(Constants.ids.CLAW);
  private AnalogInput sensor = new AnalogInput(Constants.ids.ULTRASONIC_SENSOR);
  
  private Wrist wrist = Wrist.getInstance();

  private boolean isCube;

  private Claw() {
    init();
    TempManager.addMotor(motor);
  }

  private void init() {
    motor.setBrake(true);
    motor.setInverted(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Distance Sensor", getTriggered());
    SmartDashboard.putBoolean("IsCube", isCube);
  }

  public void disable() {
    motor.disable();
  }

  public void set(double percent) {
    motor.set(percent);
  }

  private void setInverted() {
    if (isCube && wrist.getMotorAngle() < 0) motor.setInverted(true);
    else motor.setInverted(false);
  }

  public void setIsCube(boolean isCube) {
    this.isCube = isCube;
  }

  public void intake() {
    setInverted();

    motor.set(1.0);
  }

  public void outtake() {
    setInverted();

    motor.set(-1.0);
  }

  public boolean getTriggered() {
    return sensor.getValue() > 250;
  }

  public double getClawLength() {
    return (isCube) ? Constants.robot.CUBE_DISTANCE : Constants.robot.TOTAL_DISTANCE;
  }

  public boolean getIsCube() {
    return isCube;
  }
}
