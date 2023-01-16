package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.drivetrain;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance;
  public static synchronized Drivetrain getInstance(){
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  private SwerveModule frontLeft = new SwerveModule(1, 2, 3, 0);

  private Drivetrain() {

  }

@Override
public void periodic() {
  SmartDashboard.putNumber("Speed Motor Velocity", frontLeft.getModuleState().speedMetersPerSecond);
  SmartDashboard.putNumber("Speed Motor Angle", frontLeft.getModuleState().angle.getDegrees());
}

}