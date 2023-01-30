package frc.robot.AidenLib.example;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.AidenLib.Timer;
import frc.robot.AidenLib.math.Data;
import frc.robot.AidenLib.math.Integral;
import frc.robot.AidenLib.math.WeightedFusion;

public class IMUFusionExample {

    public static final double accelStDev = 0.1;
    public static final double driveStDev = 0.05;

    private double[] lastVelEstimate = {0.0, 0.0, 0.0};
    private Timer timer = new Timer();

    public void periodic(Drivetrain drive, IMU imu) {
        double[] accel = imu.getAccel();
        double dt = timer.getDT();

        ArrayList<Data> vel = new ArrayList<Data>();

        vel.add(new Data(lastVelEstimate[0] + Integral.riemannSum(accel[0], dt), accelStDev));
        vel.add(new Data(lastVelEstimate[1] + Integral.riemannSum(accel[1], dt), accelStDev));
        vel.add(new Data(lastVelEstimate[2] + Integral.riemannSum(accel[2], dt), accelStDev));

        ChassisSpeeds speeds = drive.getChassisSpeeds();

        vel = WeightedFusion.calculateParameterized(
            List.of(
                List.of(vel.get(0), new Data(speeds.vxMetersPerSecond, driveStDev)),
                List.of(vel.get(1), new Data(speeds.vyMetersPerSecond, driveStDev)),
                List.of(vel.get(2), new Data(0.0, driveStDev))
            )
        );

        lastVelEstimate[0] = vel.get(0).value;
        lastVelEstimate[1] = vel.get(1).value;
        lastVelEstimate[2] = vel.get(2).value;
    }
}
