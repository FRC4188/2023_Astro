package frc.robot.commands.Claw;

import frc.robot.subsystems.Claw;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class CubeClaw extends CommandBase {
  private Claw claw = Claw.getInstance();
  private double power;
  private TalonSRXControlMode mode;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CubeClaw(TalonSRXControlMode mode, double power) {
    addRequirements(claw);
    this.power = power;
    this.mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    claw.set(mode, power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.set(mode, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
