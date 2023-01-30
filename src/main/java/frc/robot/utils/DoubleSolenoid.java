package frc.robot.utils;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class DoubleSolenoid {

  private Solenoid A;
  private Solenoid B;

  public DoubleSolenoid(int chnA, int chnB) {
    A = new Solenoid(PneumaticsModuleType.CTREPCM, chnA);
    B = new Solenoid(PneumaticsModuleType.CTREPCM, chnB);
  }

  public void set(boolean state) {
    A.set(state);
    B.set(!state);
  }

  public boolean get() {
    return A.get();
  }

  public void off() {
    A.set(false);
    B.set(false);
  }
}
