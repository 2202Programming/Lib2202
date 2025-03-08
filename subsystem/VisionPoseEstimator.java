package frc.lib2202.subsystem;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.util.VisionWatchdog;

// Swerve Drive Train (drivetrain) must be created before Swerve-PoseEstimator

public class VisionPoseEstimator extends SubsystemBase implements OdometryInterface
{
    // set true if we found everything needed, otherwise this system is disabled
    final boolean correct_config;

    // This connects us to whatever gyro is being used for robot heading, configured
    // in RobotSpecs
    final IHeadingProvider gyro;
    final DriveTrainInterface drivetrain;  
    final OdometryInterface m_odometry;    // read-only here, updated in drivetrain
    final SwerveDriveKinematics kinematics; // const matrix based on chassis geometry, get from drivetrain
    SwerveModulePosition[] meas_pos; // provided by drivetrain

    Pose2d m_odoPose;  //based on odometry

    final VisionWatchdog watchdog;
    final BaseLimelight limelight;

    Matrix<N3, N1> visionMeasurementStdDevs = new Matrix<N3, N1>(N3.instance, N1.instance);

    // Bearing calcs (TBD)
    // private double currentBearing = 0;
    //private double filteredBearing = 0;
    //private double filteredVelocity = 0;

    // Creates a new Single-Pole IIR filter
    // Filter Time constant is 0.1 seconds
    // Period is 0.02 seconds - this is the standard FRC main loop period
    //private LinearFilter bearingFilter = LinearFilter.singlePoleIIR(0.1, Constants.DT);
    //private LinearFilter velocityFilter = LinearFilter.singlePoleIIR(0.1, Constants.DT);

    boolean visionPoseUsingRotation = true; // read from drivetrain.useVisionRotation()
    boolean visionPoseEnabled = true;

    final SwerveDrivePoseEstimator m_estimator;
    // monitor diffs in ll and odometry poses
    private double x_diff; // [m]
    private double y_diff; // [m]
    private double yaw_diff; // [deg]

    //vision systems limelight and photonvision(TBD)
    private Pose2d llPose;
    private Pose2d prev_llPose;

    // field estimate based on vision estimate llPose
    public final Field2d m_field;
    final FieldObject2d m_field_obj;
    final String m_ll_name;
    String altName;

    // no-args ctor, default timings
    public VisionPoseEstimator() {
        this(3.0, "limelight"); // typical settings
    }
    // no-args ctor, default timings
    public VisionPoseEstimator(String limelightName) {
        this(3.0, limelightName); // typical settings
    }

    public VisionPoseEstimator(double watchdog_interval, String limelightName) {
        watchdog = new VisionWatchdog(watchdog_interval);
        m_field = new Field2d();
        m_ll_name = limelightName;
        m_field_obj = m_field.getObject("VPE_" + m_ll_name);

        // other subsystems
        drivetrain = RobotContainer.getSubsystemOrNull("drivetrain");
        m_odometry = RobotContainer.getSubsystemOrNull("odometry");
        gyro = RobotContainer.getRobotSpecs().getHeadingProvider();
        limelight = RobotContainer.getSubsystemOrNull(limelightName);

        altName = limelight.getName();  //debug

        // confirm config is correct
        correct_config = drivetrain != null && gyro != null && 
                         limelight != null && m_odometry != null;

        if (drivetrain != null && m_odometry != null) {
            kinematics = drivetrain.getKinematics();
            meas_pos = drivetrain.getSwerveModulePositions();            
            m_odoPose = m_odometry.getPose();
        } else {
            // no drivetrain, set the drivetrain related final vars
            kinematics = null;
            m_odoPose = new Pose2d();
            meas_pos = null;
        }
        
        //set initial values to odometry based m_odoPose
        llPose = prev_llPose = m_odoPose;
        
        if (correct_config) {
            // Estimators
            m_estimator = initializeEstimator();
            // start the network monitor
            this.new VisionPoseEstimatorMonitorCmd();
        }
        else {
            m_estimator = null;
        }
    } // ctor

    @Override
    public void periodic() {
        if (!correct_config) return;

        m_odoPose = m_odometry.getPose();
        meas_pos = drivetrain.getSwerveModulePositions();
        llPose = updateEstimator();
        m_field_obj.setPose(llPose);

        // compare llPose and odometry pose for reporting       
        x_diff = Math.abs(llPose.getX() - m_odoPose.getX());
        y_diff = Math.abs(llPose.getY() - m_odoPose.getY());
        yaw_diff = Math.abs(llPose.getRotation().getDegrees() - m_odoPose.getRotation().getDegrees());
    }

    // helper functions
    SwerveDrivePoseEstimator initializeEstimator() {
        /*
         * Here we create SwerveDrivePoseEstimator so that we can fuse odometry readings.
         * The numbers used below are robot specific, and should be tuned.
         * 
         * TODO - add PID config to RobotSpecs
         * TODO - std seem really high for vision, esp the heading
         */
        var estimator = new SwerveDrivePoseEstimator(
                kinematics,
                gyro.getRotation2d(),
                this.meas_pos,
                this.m_odoPose, 
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2)), // std x,y, heading from odmetry [m,deg]  5
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10))); // std x, y heading from vision [m, deg] 30
        return estimator;
    }

    /** Updates the field relative position of the robot. */
    Pose2d updateEstimator() {
        prev_llPose = llPose;
        // let limelight sub-system decide if we are good to use estimate
        // OK if it is run only intermittantly. Uses latency of vision pose.
        if (!limelight.getRejectUpdate()) { 
            var pose = limelight.getBluePose();
            var ts = limelight.getVisionTimestamp();
            
            //wip use ll stddevs to adjust vision measurments
            double[] stddevs = getStddevs();
            visionMeasurementStdDevs.set(0, 0, stddevs[6]);   //mt2 x
            visionMeasurementStdDevs.set(0, 0, stddevs[7]);   //mt2 y
            visionMeasurementStdDevs.set(0, 0, stddevs[8]);   //mt2 deg

            m_estimator.addVisionMeasurement(pose, ts);
            if (watchdog != null)
                watchdog.update(pose, prev_llPose);
        }
        return m_estimator.update(gyro.getRotation2d(), meas_pos);       
    }

    //set drivetrain's pose if it's enabled
    // This couples odometry by forcing it to take the LL pose.
    // really we are incorporating the odometry measurements into the LLPoseEstimator.
    // Should be able to have pathing users take this estimator's pose
    @Deprecated
    void useEstimate() {
        visionPoseUsingRotation = m_odometry.useVisionRotation();
        visionPoseEnabled = m_odometry.useVisionPose();
        
        Rotation2d current_rotation = m_odoPose.getRotation();
        if (visionPoseEnabled) {           
            if(watchdog != null) watchdog.update(llPose, prev_llPose);
            if (visionPoseUsingRotation) {
                // update robot pose, include vision-based rotation
                m_odometry.setPose(llPose);
            } else {
                // update robot translation, do not update rotation
                m_odometry.setPose(new Pose2d(llPose.getTranslation(), current_rotation));
            }         
        }
    }

    public void configureGyroCallback(){
        AllianceAwareGyroReset.AddCallback(this::setAnglePose);
    }
    
    
    // see if we can get the LL stddevs for mt1[0..5] and mt2[6..11]
    double[] default_stddevs = new double[12];
    public double[] getStddevs() {
        return 
            NetworkTableInstance.getDefault().getTable(m_ll_name)
                .getEntry("stddevs").getDoubleArray(default_stddevs);
      }

    /** 
     * @return Pose2d
     */
    // Public API

    public void printVisionPose() {
        System.out.println("***VisionPose\n  X:" + llPose.getX() +
            "\n  Y:" + llPose.getY() + "\n  Rot:" + llPose.getRotation().getDegrees());
    }

    public double getDistanceToTranslation(Translation2d targetTranslation) {
        return Math.sqrt(
            Math.pow(llPose.getX() - targetTranslation.getX(), 2.0) +
            Math.pow(llPose.getY() - targetTranslation.getY(), 2.0));
    }
    
    @Override
    public void setPose(Pose2d newPose) {
        m_odoPose = newPose;
        // set everything to new pose
        gyro.setHeading(m_odoPose.getRotation()); 
        m_odometry.setPose(m_odoPose);
        
        // drive positions could be cleare, re-read them
        meas_pos = drivetrain.getSwerveModulePositions();
        // set our estimators new pose with current drivetrains wheel meas_pos
        m_estimator.resetPosition(gyro.getHeading(), meas_pos, m_odoPose);
        llPose = m_estimator.getEstimatedPosition();  //note - llpose should be same 
    }
    
    @Override
    public void setAnglePose(Rotation2d rot) {
        setPose(new Pose2d(m_odoPose.getTranslation(), rot));
    }
    @Override
    public Pose2d getPose() {
        return llPose;
    }
    @Override
    public void printPose() {
        System.out.println("***VisionPoseEstimator " + m_ll_name + " X:" + llPose.getX() +
        ", Y:" + llPose.getY() +
        ", Rot:" + llPose.getRotation().getDegrees());
    }
    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }


    /*
     * Watcher for SwervePoseEstimator and its vision data.
     *
     * Only watches high level data, for module details see the tables for each of
     * the modules.
     */
    public class VisionPoseEstimatorMonitorCmd extends WatcherCmd {

        // final private NetworkTable table;
        NetworkTableEntry nt_x_diff;
        NetworkTableEntry nt_y_diff;
        NetworkTableEntry nt_yaw_diff;

        NetworkTableEntry est_ll_pose_x;
        NetworkTableEntry est_ll_pose_y;
        NetworkTableEntry est_ll_pose_h;
        NetworkTableEntry est_pv_pose_x;
        NetworkTableEntry est_pv_pose_y;
        NetworkTableEntry est_pv_pose_h;

        Pose2d ll_pose;
        Pose2d pv_pose;

        public VisionPoseEstimatorMonitorCmd() {
        }

        @Override
        public String getTableName() {
            return VisionPoseEstimator.class.getSimpleName();
        }

        @Override
        public void ntcreate() {
            NetworkTable MonitorTable = getTable();
            est_ll_pose_x = MonitorTable.getEntry("/LL/X");
            est_ll_pose_y = MonitorTable.getEntry("/LL/Y");
            est_ll_pose_h = MonitorTable.getEntry("/LL/Heading");

            //est_pv_pose_x = MonitorTable.getEntry("/PV/X");
            //est_pv_pose_y = MonitorTable.getEntry("/PV/Y");
            //est_pv_pose_h = MonitorTable.getEntry("/PV/Heading");

            // Network Table setup
            nt_x_diff = MonitorTable.getEntry("/compareLLOdo/diffX");
            nt_y_diff = MonitorTable.getEntry("/compareLLOdo/diffY");
            nt_yaw_diff = MonitorTable.getEntry("/compareLLOdo/diffHeading");
        }

        // Network Table Monitoring
        @Override
        public void ntupdate() {
            SmartDashboard.putData("Field_vision", m_field);

            if (ll_pose != null) {
                est_ll_pose_x.setDouble(ll_pose.getX());
                est_ll_pose_y.setDouble(ll_pose.getY());
                est_ll_pose_h.setDouble(ll_pose.getRotation().getDegrees());
            }
            if (pv_pose != null) {
            //    est_pv_pose_x.setDouble(pv_pose.getX());
            //    est_pv_pose_y.setDouble(pv_pose.getY());
            //    est_pv_pose_h.setDouble(pv_pose.getRotation().getDegrees());
            }

            // vision pose updating NTs
            nt_x_diff.setDouble(x_diff);
            nt_y_diff.setDouble(y_diff);
            nt_yaw_diff.setDouble(yaw_diff);
        }

    } // monitor cmd class

    // https://www.chiefdelphi.com/t/limelight-odometry-question/433311/6
// public void updatePoseEstimatorWithVisionBotPose() {
//     PoseLatency visionBotPose = m_visionSystem.getPoseLatency();
//     // invalid LL data
//     if (visionBotPose.pose2d.getX() == 0.0) {
//       return;
//     }

//     // distance from current pose to vision estimated pose
//     double poseDifference = m_poseEstimator.getEstimatedPosition().getTranslation()
//         .getDistance(visionBotPose.pose2d.getTranslation());

//     if (m_visionSystem.areAnyTargetsValid()) {
//       double xyStds;
//       double degStds;
//       // multiple targets detected
//       if (m_visionSystem.getNumberOfTargetsVisible() >= 2) {
//         xyStds = 0.5;
//         degStds = 6;
//       }
//       // 1 target with large area and close to estimated pose
//       else if (m_visionSystem.getBestTargetArea() > 0.8 && poseDifference < 0.5) {
//         xyStds = 1.0;
//         degStds = 12;
//       }
//       // 1 target farther away and estimated pose is close
//       else if (m_visionSystem.getBestTargetArea() > 0.1 && poseDifference < 0.3) {
//         xyStds = 2.0;
//         degStds = 30;
//       }
//       // conditions don't match to add a vision measurement
//       else {
//         return;
//       }

//       m_poseEstimator.setVisionMeasurementStdDevs(
//           VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
//       m_poseEstimator.addVisionMeasurement(visionBotPose.pose2d,
//           Timer.getFPGATimestamp() - visionBotPose.latencySeconds);
//     }
//   }

}
