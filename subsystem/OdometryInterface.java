package frc.lib2202.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


public interface OdometryInterface {
    // core odometry API
    public void setPose(Pose2d newPose);   // new x,y,heading
    public void setAnglePose(Rotation2d rot); // new heading, keeps x,y
    public void setTranslation(Translation2d newPosition); // new xy, no gyro reset
    public Pose2d getPose();
    public void printPose();
    public double getDistanceToTranslation(Translation2d targetTranslation);

    // also provide kinematics matrix
    public SwerveDriveKinematics getKinematics();
    
    //public  SwerveDriveOdometry getOdometry(); // should not need, use api

    // Extend API for vision pose interface - Can this be cleaned up?
    // For some odometry these are non-functional
    public default void enableVisionPose() {};
    public default void disableVisionPoseRotation() {};
    public default void enableVisionPoseRotation() {};
    public default boolean useVisionRotation() {return false;};
    public default boolean useVisionPose() {return false;};    
}
