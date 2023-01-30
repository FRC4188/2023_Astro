package frc.robot.utils;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class ContinuousTrapezoid {
  private final Constraints constraints;

  public ContinuousTrapezoid(Constraints constraints) {
    this.constraints = constraints;
  }

  public State calculate(double current, double goal) {
    State cState = new State(current, 0.0);
    State gState = new State(goal, 0.0);
    TrapezoidProfile profile = new TrapezoidProfile(constraints, gState, cState);

    return profile.calculate(0.02);
  }
}
