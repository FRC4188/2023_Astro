package frc.robot.subsystems.claw;

import csplib.motors.CSP_Talon;
import csplib.utils.TempManager;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  private static Claw instance;

  public static synchronized Claw getInstance() {
    if (instance == null) instance = new Claw();
    return instance;
  }

  private CSP_Talon motor = new CSP_Talon(Constants.ids.CLAW);
  private AnalogInput sensor = new AnalogInput(Constants.ids.ULTRASONIC_SENSOR);

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
    SmartDashboard.putBoolean("isCube", getIsCube());
  }

  public void disable() {
    motor.disable();
  }

  public void set(double percent) {
    motor.set(percent);
  }

  private void setInverted() {
    if (isCube) motor.setInverted(true);
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

  public boolean getIsCube() {
    return isCube;
  }
}
