// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.command.swerve;

import static frc.lib2202.Constants.DEGperRAD;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
  private final DriveTrainInterface drivetrain;
  private final OdometryInterface odometry;
  private final PIDController pid;
  private final double kp = 0.05;
  private final double ki = 0.0;
  private final double kd = 0.0;
  private final double pos_tol = 10.0;
  private final double vel_tol = 1.0;
  private SwerveDriveKinematics kinematics;
  private Pose2d currentPose;
  private double targetRot;
  private SwerveModuleState[] outputModuleState;
  
  Translation2d target; // Position want to face to

  //Alliance 
  final Translation2d redTarget;
  final Translation2d blueTarget;
  final private Timer timer;

  /** Creates a new RotateTo. */
   public RotateTo(Translation2d redTarget, Translation2d blueTarget){
    this(redTarget, blueTarget, 4.0);
   }

  public RotateTo(Translation2d redTarget, Translation2d blueTarget, double timeout) {
    this.redTarget = redTarget;
    this.blueTarget = blueTarget;

    drivetrain = RobotContainer.getSubsystem("drivetrain");
    //use vision based odometry, if it exists
    OdometryInterface odo = RobotContainer.getSubsystemOrNull("vision_odo");
    this.odometry = (odo != null)  ? odo : RobotContainer.getSubsystem("odometry");

    addRequirements(drivetrain);
    pid = new PIDController(kp, ki, kd);
    pid.enableContinuousInput(-180.0, 180.0);
    pid.setTolerance(pos_tol, vel_tol);
    kinematics = drivetrain.getKinematics();
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = (DriverStation.getAlliance().get() == Alliance.Blue) ? blueTarget : redTarget;
   
    timer.restart();
    currentPose = odometry.getPose();
    double dy = target.getY() - currentPose.getY();
    double dx = target.getX() - currentPose.getX();
    targetRot = Math.atan2(dy, dx) * DEGperRAD; // [deg] heading to TARGET
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculate();
    drivetrain.drive(outputModuleState);
  }

  private void calculate() {
    currentPose = odometry.getPose();
    outputModuleState = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        pid.calculate(currentPose.getRotation().getDegrees(), targetRot),
        currentPose.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint() || timer.hasElapsed(4);
  }

}
