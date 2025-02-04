package frc.lib2202.subsystem.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveTrainInterface extends SubsystemBase {

    public abstract void autoPoseSet(Pose2d pose);

    public abstract void printPose();

    public abstract void enableVisionPose();

    public abstract void resetAnglePose(Rotation2d rot);

    public abstract void disableVisionPoseRotation();

    public abstract void enableVisionPoseRotation();

    public abstract SwerveDriveKinematics getKinematics();

    public abstract Pose2d getPose();

    public abstract void drive(SwerveModuleState[] states);

    public abstract void stop();

    public abstract void setPose(Pose2d pose);

    public abstract SwerveModulePosition[] getSwerveModulePositions();

    public abstract SwerveDriveOdometry getOdometry();

    public abstract ChassisSpeeds getFieldRelativeSpeeds();

    public abstract boolean useVisionRotation();

    public abstract boolean useVisionPose();

    public abstract ChassisSpeeds getChassisSpeeds();

    public abstract double getDistanceToTranslation(Translation2d targetTranslation);

}
