package frc.robot.AidenLib.example;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.AidenLib.control.PoseEstimator3D;
import frc.robot.AidenLib.control.RotationEstimator3D;
import frc.robot.AidenLib.math.Data;

public class EstimatorExample {

    public static final double imuStdDev = 0.1;
    public static final double driveStDev = 0.1;

    private RotationEstimator3D rotEstimator = new RotationEstimator3D(imuStdDev);
    private PoseEstimator3D poseEstimator = new PoseEstimator3D(() -> driveStDev);

    public void periodic(Drivetrain drive, IMU imu, Camera cam) {
        Pose3d camPose = cam.getPose();
        Rotation3d camRot = camPose.getRotation();
        double[] stDevs = cam.getStdDevs();
        Rotation3d rotEstimate = rotEstimator.estimate(imu.getHeading(),
            new LinkedList<Data>(List.of(new Data(camRot.getZ(), stDevs[5]))),
            new LinkedList<Data>(List.of(new Data(camRot.getY(), stDevs[4]))),
            new LinkedList<Data>(List.of(new Data(camRot.getX(), stDevs[3])))
        );

        Pose3d poseEstimate = poseEstimator.estimate(drive.getChassisSpeeds(), rotEstimate,
        new LinkedList<Data>(List.of(new Data(camPose.getX(), stDevs[2]))),
        new LinkedList<Data>(List.of(new Data(camPose.getY(), stDevs[1]))),
        new LinkedList<Data>(List.of(new Data(camPose.getZ(), stDevs[0])))
        );
    }
}
