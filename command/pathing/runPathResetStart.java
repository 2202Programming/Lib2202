// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.command.pathing;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.OdometryInterface;

//This command is a hack to reset our current pose to path start pose
//since we can't figure out how to make autobuilder do this
public class runPathResetStart extends Command {
  
  Command pathCommand;
  final String pathName;
  final OdometryInterface odometry;

  public runPathResetStart(String pathName) {
    this("odometry", pathName);
  }

  public runPathResetStart(String odometryName, String pathName) {
    odometry = RobotContainer.getSubsystem(odometryName);
    this.pathName = pathName;
  }

  @Override
  public void initialize() {

    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(pathName);     
      Pose2d startPose = path.getPathPoses().get(0);
      /* was  
      PathPoint startPoint = path.getPoint(0);
      Pose2d startPose = new Pose2d(
          new Translation2d(startPoint.position.getX(), startPoint.position.getY()),
          new Rotation2d(0.0) );
      */
  
      pathCommand = AutoBuilder.followPath(path);
  
      odometry.setPose(startPose);
      new InstantCommand(odometry::printPose).schedule();
      pathCommand.schedule();
    
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      DriverStation.reportError("Big oops: No path cmd scheduled during initialize()", null);
    }
   }

  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("***RunPathResetStart Ended, current pose:");
    odometry.printPose();
    odometry.enableVisionPose();
  }
}
