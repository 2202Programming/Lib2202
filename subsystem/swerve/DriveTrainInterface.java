package frc.lib2202.subsystem.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveTrainInterface extends SubsystemBase {

    public abstract void drive(SwerveModuleState[] states);

    public abstract void stop();

    public abstract SwerveModulePosition[] getSwerveModulePositions();

    public abstract ChassisSpeeds getChassisSpeeds();

    //TODO - how to deal with swerve or non-swerve platforms... @Nathan
    public abstract SwerveDriveKinematics getKinematics();

    public abstract ChassisSpeeds getFieldRelativeSpeeds();
    
    public abstract void setPositions(double position);

    public abstract void driveRobotRelative(ChassisSpeeds chassisSpeed);
}
