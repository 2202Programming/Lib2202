package frc.lib2202.command.swerve;

import static frc.lib2202.Constants.DEGperRAD;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.Constants;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.lib2202.subsystem.ILimelight;
import frc.lib2202.subsystem.LimelightHelpers.RawFiducial;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.hid.HID_Subsystem;

/**
 * 
  Driver controls the robot using field centric driving, but rotation will face the target.
    X,Y, Rotation
**/
public class TargetCentricDrive extends Command {

  public enum state {
    Init("Init"),
    BlindTrack("BlindTrack"), // uses vision/odometry to track
    TagTrack("TagTrack"); // we can see the tag, use it
    private String name;

    private state(String name) {
      this.name = name;
    }

    public String toString() {
      return name;
    }
  }

  private state currentState;
  final DriveTrainInterface drivetrain;
  final OdometryInterface odometry;
  final ILimelight limelight;

  final SwerveDriveKinematics kinematics;
  final HID_Subsystem dc;
  final RobotLimits limits;

  // Alliance aware targets either Translation2d or a AprilTag
  final Translation2d redTarget;
  final AprilTag redTag;
  final Translation2d blueTarget;
  final AprilTag blueTag;

  // Limelight PID - to command rotation to target
  PIDController blindPid;
  final double blindPid_kp = 3.0;
  final double blindPid_ki = 0.0;
  final double blindPid_kd = 0.0;

  double targetRot;
  Pose2d currentPose;
  Translation2d target; // target location x,y to face to
  AprilTag  targetTag;  // will be non-zero if looking for AprilTag

  // odometery PID
  PIDController centeringPid;
  double centering_kP = 2.0; // used to be 3.5 when we were in degrees
  double centering_kI = 0;
  double centering_kD = 0;
  double centeringPidOutput = 2.0;
  double vel_tol = 1.0 / 57.3; // [rad/sec]
  double pos_tol_blind = 5.0 / 57.3; // [rad]
  double pos_tol_tag = 2.5;
  double max_rot_rate = 45.0; // [deg/s]
  double min_rot_rate = 6.0;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot_cmd;
  Rotation2d currrentHeading;
  SwerveModuleState[] output_states;
  boolean hasTarget = false;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  // TagID based tracking
  public TargetCentricDrive(AprilTag redTag, AprilTag blueTag) {
    this(null, null, redTag, blueTag, "limelight");
  }

  // General Translation2d tracking
  public TargetCentricDrive(Translation2d redTarget, Translation2d blueTarget) {
    this(redTarget, blueTarget, null, null, "limelight");
  }

  // command deals with either tagID or translation tracking
  public TargetCentricDrive(Translation2d redTarget, Translation2d blueTarget,
                            AprilTag redTag,         AprilTag blueTag, String limelightName) {
    // deal with tag or given Translations, get tranlation2d from tags if given
    this.redTarget  = (redTarget != null)  ? redTarget  : new Translation2d(redTag.pose.getX(), redTag.pose.getY());
    this.blueTarget = (blueTarget != null) ? blueTarget : new Translation2d(blueTag.pose.getX(), blueTag.pose.getY());
    this.blueTag = blueTag;
    this.redTag = redTag;

    this.dc = RobotContainer.getSubsystem("DC"); // driverControls
    this.drivetrain = RobotContainer.getSubsystem("drivetrain");
    // use vision based odometry, if it exists
    OdometryInterface odo = RobotContainer.getSubsystemOrNull("vision_odo");
    this.odometry = (odo != null) ? odo : RobotContainer.getSubsystem("odometry");

    this.limits = RobotContainer.getRobotSpecs().getRobotLimits();
    this.kinematics = drivetrain.getKinematics();
    this.limelight = RobotContainer.getSubsystem(limelightName);

    addRequirements(drivetrain); // This means we area read-only for everything but drivetrain

    // PID for when tag is in view
    centeringPid = new PIDController(centering_kP, centering_kI, centering_kD);
    centeringPid.setTolerance(pos_tol_tag, vel_tol);

    // PID for when tag is not visable
    blindPid = new PIDController(blindPid_kp, blindPid_ki, blindPid_kd); // [rad]
    blindPid.enableContinuousInput(-Math.PI, Math.PI); // [rad]
    blindPid.setTolerance(pos_tol_blind, vel_tol); // Not being used
  }

  @Override
  public void initialize() {
    target = (DriverStation.getAlliance().get() == Alliance.Blue) ? blueTarget : redTarget;
    targetTag = (DriverStation.getAlliance().get() == Alliance.Blue) ? blueTag : redTag;
    currentState = state.Init;
    SmartDashboard.putString("TargetCentricDrive/State", currentState.toString());
  }

  @Override
  public void execute() {
    double tagXfromCenter = 0.0;
    this.hasTarget = false;
    currentPose = odometry.getPose();
    currrentHeading =  currentPose.getRotation();

    // see if we have a target
    if (targetTag != null) {
      tagXfromCenter = checkForTarget(targetTag.ID); // updates tagXfromCenter, this.hasTarget
    }
    
    if (this.hasTarget) {
      currentState = state.TagTrack;
    } else {
      currentState = state.BlindTrack; //use odometry alone
    }

    this.rot_cmd = calculateRotFromOdometery(); // always feed PID, even if rot gets overwritten later.

    switch (currentState) {
      case TagTrack:      
        this.rot_cmd = calculateRotFromTarget(tagXfromCenter); // has note, can see target tag, close loop via limelight
        break;

      case Init: // should never get here
        System.out.println("***Impossible state reached in TargetCentricDrive***");
        break;

      case BlindTrack:
        // can't see target Tag, so use odometery for rot (already run)
        break;
    }

    calculate(); // used to calculate X and Y from joysticks, and rotation from one of methods                
    drivetrain.drive(output_states);
    SmartDashboard.putString("TargetCentricDrive/State", currentState.toString());
    SmartDashboard.putNumber("TargetCentricDrive/TagX", tagXfromCenter);
    SmartDashboard.putBoolean("TargetCentricDrive/hasTarget", this.hasTarget);
    SmartDashboard.putNumber("TargetCentricDrive/rot_cmd_deg", rot_cmd * DEGperRAD);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("TargetCentricDrive/State", "ended");
    drivetrain.stop();
  }

  private double calculateRotFromOdometery() {
    if (target == null) {
      //incase no target was set, unlikely.
      return 0.0;
    }

    targetRot = Math.atan2(currentPose.getY() - target.getY(),
                           currentPose.getX() - target.getX()); // [-pi, pi]

    SmartDashboard.putNumber("TargetCentricDrive/Odo_target", targetRot * DEGperRAD);
    // use pid to calculate rot_cmd[rad/s] using targetRot angle as setpoint
    double rot_cmd = blindPid.calculate(currentPose.getRotation().getRadians(), targetRot);  //[rad/s]
    return rot_cmd; // [rad/s]
  }

  private double calculateRotFromTarget(double tagXfromCenter) {
    // use distance from center of tag
    centeringPidOutput = centeringPid.calculate(tagXfromCenter, 0.0);
    double min_rot = Math.signum(centeringPidOutput) * min_rot_rate;
    double rot_cmd = MathUtil.clamp(centeringPidOutput + min_rot, -max_rot_rate, max_rot_rate) / Constants.DEGperRAD;
    return rot_cmd; // [rad/s]
  }

  private void calculate() { // lets use arguments and returns please
    // X and Y from joysticks; rot from previous calculations
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * limits.kMaxSpeed;
    ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * limits.kMaxSpeed;

    // Clamp speeds from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -limits.kMaxSpeed, limits.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -limits.kMaxSpeed, limits.kMaxSpeed);

    // convert field centric speeds to robot centric
    // if on red alliance you're looking at robot from opposite. 
    // Pose is in blue coordinates so flip if red
    ChassisSpeeds tempChassisSpeed = (DriverStation.getAlliance().get().equals(Alliance.Blue))
        ? ChassisSpeeds.fromFieldRelativeSpeeds( xSpeed,  ySpeed, rot_cmd, currrentHeading)
        : ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, rot_cmd, currrentHeading); 
    this.output_states = kinematics.toSwerveModuleStates(tempChassisSpeed);
  }

  private double checkForTarget(int tagID) {
    double tagXfromCenter = 0.0;
    this.hasTarget = false;
    // protect if not using a tagID
    if (tagID <= 0) {
      return tagXfromCenter;
    }

    RawFiducial tag = limelight.checkForTarget(tagID);
    if (tag != null) {
      tagXfromCenter = tag.txnc;
      this.hasTarget = true;
    }
    return tagXfromCenter;
  }

}