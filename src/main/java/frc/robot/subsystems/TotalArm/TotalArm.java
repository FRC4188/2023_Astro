package frc.robot.subsystems.TotalArm;

import frc.robot.Constants.arm.Wrist;
import frc.robot.subsystems.arm.Elevator;
import frc.robot.subsystems.arm.ShoulderJoint;

public class TotalArm {

    private static TotalArm instance = null;
    ShoulderJoint shoulder = ShoulderJoint.getInstance();
    Elevator elevator = elevator.getInstance();
    Wrist wrist = wrist.getInstance();

    public static synchronized TotalArm getInstance(){
    if (TotalArm == null)
      TotalArm = new TotalArm();
    return TotalArm;
  }

  public double C{
    double C = Math.sqrt(elevator.setAngleWithPID()+Constants.WRIST_LENGTH-2*(elevator.setAngleWithPID())*Constants.WRIST_LENGTH*Math.cos(180-wrist.getAngle()));

  }

  public double Phi(){
    return(Math.asin(constants.arm.TotalWRIST_LENGTH*Math.sin(180-Wrist.getAngle)/C)+ShoulderJoint.getAngle());
  }

  public Pose3d CalculatePose3d{
    return Pose3d(c*Math.sin(pigeon angle)*Math.cos(Phi), c*Math.sin(pigeon angle)*Math.cos(Phi), C*Math.cos(pigeon));
  }


  











}