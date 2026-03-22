// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.command.swerve;

import static frc.lib2202.Constants.DEGperRAD;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;

public class RotateTo extends Command {
  // devices
  private final DriveTrainInterface drivetrain; // requirement
  private final OdometryInterface odometry;
  private final PIDController pid;

  // parameters
  private final double kp = 2.5; // [deg/s / deg-error]
  private final double ki = 0.0;
  private final double kd = 0.0;
  private final double pos_tol = 2.0; // [deg]
  private final double vel_tol = 1.0; // [deg/s]

  private final SwerveDriveKinematics kinematics;
  private Pose2d currentPose;
  private double targetRot;
  private Rotation2d targetRot2d;
  private SwerveModuleState[] outputModuleState;

  Translation2d target; // Position to face, set at init based on alliance

  // Alliance
  final Translation2d redTarget;
  final Translation2d blueTarget;
  final Rotation2d redTargetRot;
  final Rotation2d blueTargetRot;
  final Timer timer;
  final double timeout;
  final SwerveModuleState[] zero_sms;

  /** Creates a new RotateTo. */
  public RotateTo(Translation2d redTarget, Translation2d blueTarget) {
    this(redTarget, blueTarget, 4.0);
  }

  // Alliance aware rotation only by degrees (no targets)
  public RotateTo(Rotation2d redTargetRot, Rotation2d blueTargetRot, double timeout) {

    this.redTarget = null;
    this.blueTarget = null;
    this.timeout = timeout;
    this.blueTargetRot = blueTargetRot;
    this.redTargetRot = redTargetRot;
    timer = new Timer();

    drivetrain = RobotContainer.getSubsystem("drivetrain");
    // use vision based odometry, if it exists
    OdometryInterface odo = RobotContainer.getSubsystemOrNull("vision_odo");
    this.odometry = (odo != null) ? odo : RobotContainer.getSubsystem("odometry");

    addRequirements(drivetrain);

    // pid for driving rotation angle
    pid = new PIDController(kp, ki, kd);
    pid.enableContinuousInput(-180.0, 180.0);
    pid.setTolerance(pos_tol, vel_tol);
    kinematics = drivetrain.getKinematics();
    zero_sms = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Don't care alliance rotation target
  public RotateTo(Rotation2d targetRot) {
    this(targetRot, targetRot, 4.0);
  }

  public RotateTo(Translation2d redTarget, Translation2d blueTarget, double timeout) {
    this.redTarget = redTarget;
    this.blueTarget = blueTarget;
    this.timeout = timeout;
    this.blueTargetRot = null;
    this.redTargetRot = null;
    timer = new Timer();

    drivetrain = RobotContainer.getSubsystem("drivetrain");
    // use vision based odometry, if it exists
    OdometryInterface odo = RobotContainer.getSubsystemOrNull("vision_odo");
    this.odometry = (odo != null) ? odo : RobotContainer.getSubsystem("odometry");

    addRequirements(drivetrain);

    // pid for driving rotation angle
    pid = new PIDController(kp, ki, kd);
    pid.enableContinuousInput(-180.0, 180.0);
    pid.setTolerance(pos_tol, vel_tol);
    kinematics = drivetrain.getKinematics();
    zero_sms = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // allow pid parameters to change
  public RotateTo setP(double kp) {
    this.pid.setP(kp);
    return this;
  }

  public RotateTo setI(double ki) {
    this.pid.setI(ki);
    return this;
  }

  public RotateTo setD(double kd) {
    this.pid.setD(kd);
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = (DriverStation.getAlliance().get() == Alliance.Blue) ? blueTarget : redTarget;
    targetRot2d = (DriverStation.getAlliance().get() == Alliance.Blue) ? blueTargetRot : redTargetRot;
    currentPose = odometry.getPose();
    if (target != null) {
      double dy = target.getY() - currentPose.getY();
      double dx = target.getX() - currentPose.getX();
      targetRot = Math.atan2(dy, dx) * DEGperRAD; // [deg] heading to TARGET
    } else {
      targetRot = targetRot2d.getDegrees();
    }
    timer.restart();
    System.out.println("RotateTo -> " + targetRot + " [deg]" + " from " + currentPose.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculate();
    drivetrain.drive(outputModuleState);
  }

  private void calculate() {
    currentPose = odometry.getPose();
    // keep PID units in degree/s, convert to rad/s for swerve
    double rot_cmd_rads = pid.calculate(currentPose.getRotation().getDegrees(), targetRot) / DEGperRAD;
    outputModuleState = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        rot_cmd_rads, currentPose.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(zero_sms); // stop moving
    timer.stop();
    System.out.println("RotateTo Done " + " ended at -> "
        + currentPose.getRotation().getDegrees()
        + " time=" + timer.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint() || timer.hasElapsed(timeout);
  }

}
