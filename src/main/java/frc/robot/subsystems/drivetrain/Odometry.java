package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utils.math.Integral;

public class Odometry {
    
    private Integral xPos = null;
    private Integral yPos = null;
    private Rotation2d heading = null;

    public Odometry(Pose2d start) {
        xPos = new Integral(start.getX());
        yPos = new Integral(start.getY());
        heading = start.getRotation();
    }

    public void update(ChassisSpeeds speeds, Rotation2d gyro) {
        double heading = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond) + gyro.getRadians();
        double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        xPos.sample(speed * Math.cos(heading));
        yPos.sample(speed * Math.sin(heading));
        this.heading = gyro;
    }

    public Pose2d getPose() {
        return new Pose2d(xPos.getValue(), yPos.getValue(), heading);
    }

    public void setPose(Pose2d pose) {
        xPos = new Integral(pose.getX());
        yPos = new Integral(pose.getY());
        heading = pose.getRotation();
    }
}