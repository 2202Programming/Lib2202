// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.command.swerve;

import static frc.lib2202.Constants.DEGperRAD;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.ILimelight;
import frc.lib2202.subsystem.LimelightHelpers.RawFiducial;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;

public class RotateUntilSeeTags extends Command {
  final DriveTrainInterface drivetrain;
  final private OdometryInterface odometry;
  final ILimelight limelight;

  private final PIDController pid;
  private final double kp = 0.04;  //[deg/s / deg-error]
  private final double ki = 0.0;
  private final double kd = 0.0;
  private final double pos_tol = 5.0; // [deg]
  private final double vel_tol = 1.0; // [deg/s]

  private SwerveDriveKinematics kinematics;
  private Pose2d currentPose;
  private double targetRot;
  private SwerveModuleState[] outputModuleState;

  AprilTag target; // Position want to face to
  private final Timer timer;
  double timeout;

  //Alliance 
  final AprilTag redTarget;
  final AprilTag blueTarget;

  public RotateUntilSeeTags(AprilTag redTag, AprilTag blueTag) {
    this(redTag, blueTag, 3.0);
  }

  /** Creates a new RotateTo. */
  public RotateUntilSeeTags(AprilTag redTag, AprilTag blueTag, double timeout) {
    this.redTarget = redTag;
    this.blueTarget = blueTag;
    this.timeout = timeout;
   
    drivetrain = RobotContainer.getSubsystem("drivetrain");
    limelight = RobotContainer.getSubsystem("limelight");
    //use vision based odometry, if it exists
    OdometryInterface odo = RobotContainer.getSubsystemOrNull("vision_odo");
    odometry = (odo != null)  ? odo : RobotContainer.getSubsystem("odometry");

    addRequirements(drivetrain);

    // rotation pid [deg]
    pid = new PIDController(kp, ki, kd);
    pid.enableContinuousInput(-180.0, 180.0);
    pid.setTolerance(pos_tol, vel_tol);
    kinematics = drivetrain.getKinematics();
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("***RotateIUntilSeeTags: Init...");
    target = (DriverStation.getAlliance().get() == Alliance.Blue) ? blueTarget : redTarget;    

    timer.restart();
    currentPose = odometry.getPose();
    targetRot = Math.atan2(currentPose.getY() - target.pose.getY(),
                currentPose.getX() - target.pose.getX()) * DEGperRAD ;  // [-pi, pi] -> [180 deg]
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = odometry.getPose();
    double rot_cmd = pid.calculate(currentPose.getRotation().getDegrees(), targetRot);
    //no x or y motion, rotate based on pid error to targetRot
    var cs = ChassisSpeeds.fromFieldRelativeSpeeds(0.0,0.0, rot_cmd, currentPose.getRotation() );
    outputModuleState = kinematics.toSwerveModuleStates(cs);
    drivetrain.drive(outputModuleState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("***RotateIUntilSeeTags: End...");
    timer.stop();
  }
   
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return checkForTarget(this.target.ID) || timer.hasElapsed(this.timeout);
  }


 /**
   * Check if limelight can see the target.
   * 
   * @param tagID tagID to check for in limelight
   * @return {@code true} if the target is found in limelight, {@code false} if
   *         not.
   */
  boolean checkForTarget(int tagID) {
    RawFiducial tag = limelight.checkForTarget(tagID);
    return (tag != null);
  }

}
