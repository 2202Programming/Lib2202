package frc.lib2202.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.lib2202.subsystem.swerve.IHeadingProvider;

public class Odometry extends SubsystemBase implements OdometryInterface {
    final DriveTrainInterface drivetrain;
    final SwerveDriveKinematics kinematics;
    final SwerveDriveOdometry odometry;
    final IHeadingProvider gyro;

    // Swerve positions
    SwerveModulePosition[] meas_pos; // distance & angle for each module

    // Outputs of this subsystem
    final Field2d m_field; // looking at code, seems there should only be one Field2d "Robot"
    final String m_odometryName;
    Pose2d m_pose; // pose based strictly on the odometry

    public Odometry() {
        this("Odometry",
            RobotContainer.getSubsystem("drivetrain"),
            RobotContainer.getSubsystem("sensors"));
    }

    public Odometry(String odometryName) {
        this(odometryName,
            RobotContainer.getSubsystem("drivetrain"),
            RobotContainer.getSubsystem("sensors"));
    }

    public Odometry(String odometryName, DriveTrainInterface drivetrain, IHeadingProvider gyro) {
        this.m_odometryName = odometryName;
        this.drivetrain = drivetrain;
        this.gyro = gyro;

        m_pose = new Pose2d(0, 0, gyro.getRotation2d());
        // Field2d tracks multiple objects by name, Robot
        m_field = new Field2d();
        kinematics = drivetrain.getKinematics();
        meas_pos = drivetrain.getSwerveModulePositions();

        odometry = new SwerveDriveOdometry(kinematics, gyro.getHeading(), meas_pos, m_pose);
        SmartDashboard.putData("FieldOdometry", m_field );
    }

    @Override
    public void periodic() {
        meas_pos = drivetrain.getSwerveModulePositions();
        m_pose = odometry.update(gyro.getHeading(), meas_pos);
        m_field.setRobotPose(m_pose);       
    }

    // sets the pose and resets the odometry states and drivetrain positions
    // and the gyro to match the pose Rotation2d
    @Override
    public void setPose(Pose2d newPose) {
        m_pose = newPose;
        // clear drivetrain position encoders
        drivetrain.setPositions(0.0);
        meas_pos = drivetrain.getSwerveModulePositions();

        //field-centric drive uses just the gyro, so reset it too even though
        //internal odometry doesn't require it. 
        gyro.setHeading(m_pose.getRotation());
        //use reset so internal odometry gyro-offset is reset
        odometry.resetPosition(gyro.getRotation2d(), meas_pos, m_pose);
    }

    // reset angle to be zero, but retain X and Y; takes a Rotation2d object
    @Override
    public void setAnglePose(Rotation2d rot) {
        setPose(new Pose2d(m_pose.getTranslation(), rot));
    }
    
    @Override
    public void setTranslation(Translation2d newPosition) {
        // update the xy, but keeps gyro unchanged
        setPose(new Pose2d(newPosition, gyro.getHeading()));
    }

    @Override
    public Pose2d getPose() {
        return m_pose;
    }

    @Override
    public void printPose() {
        System.out.println("***POSE " + m_odometryName + 
            "\n    X: " + m_pose.getX() +
            "\n    Y: " + m_pose.getY() +
            "\n  Rot: " + m_pose.getRotation().getDegrees());
    }

    @Override
    public double getDistanceToTranslation(Translation2d targetTranslation) {
        return Math.sqrt(
                Math.pow(getPose().getTranslation().getX() - targetTranslation.getX(), 2)
                        + Math.pow(getPose().getTranslation().getY() - targetTranslation.getY(), 2));
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    // Watcher
    public class OdometryWatcher extends WatcherCmd {
        // Table Entries odometry pose
        NetworkTableEntry currentX;
        NetworkTableEntry currentY;
        NetworkTableEntry currentHeading;

        public OdometryWatcher() {
        }

        @Override
        public String getTableName() {
            return m_odometryName;
        }

        @Override
        public void ntcreate() {
            NetworkTable MonitorTable = getTable();
            currentX = MonitorTable.getEntry(m_odometryName + "_x");
            currentY = MonitorTable.getEntry(m_odometryName + "_y");
            currentHeading = MonitorTable.getEntry(m_odometryName + "_h");
        }

        @Override
        public void ntupdate() {
            currentX.setDouble(m_pose.getX());
            currentY.setDouble(m_pose.getY());
            currentHeading.setDouble(m_pose.getRotation().getDegrees());
        }
    } // Watcher class

} // Odometry
