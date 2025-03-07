package frc.lib2202.command.swerve;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.OdometryInterface;

public class AllianceAwareGyroReset extends InstantCommand {
  // keep a list of callbacks because different pose estimators/gyros may need to be reset.
  static Set<Consumer<Rotation2d>> callBacks = new HashSet<Consumer<Rotation2d>>();
  
  // set of callbacks for systems that need to know when gyro reset is needed
  public static void AddCallback(Consumer<Rotation2d> consumer){
    callBacks.add(consumer);
  }

  private boolean disableVisionPoseRotation;
  final private OdometryInterface odometry;


  //resets the robot rotation/gyro, assuming robot is facing AWAY from driver, and will use alliance
  //to determine if that means 0 or 180 degrees.
  //Also has option to disable future vision rotation updates (and just rely on gryo going forward)
  public AllianceAwareGyroReset(boolean disableVisionRotation) {
    this.disableVisionPoseRotation = disableVisionRotation;
    this.odometry = RobotContainer.getSubsystem("odometry");

    if (this.odometry != null)
      AddCallback(odometry::setAnglePose);
  }

  public AllianceAwareGyroReset(){
    this(true);   //vision no longer part of basic odometry
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rotation2d direction;
    if (DriverStation.getAlliance().get() == Alliance.Blue){
      direction = Rotation2d.fromDegrees(0.0);
    }
    else{
      direction = Rotation2d.fromDegrees(180.0);//away from red driverstation is 180 degrees
    }

    // call everyone that cares about gyro reset
    for (Consumer<Rotation2d> consumer : callBacks) {
      consumer.accept(direction);
    }
    // TODO -decouple this behavior
    if(disableVisionPoseRotation){
      odometry.disableVisionPoseRotation();
    }
    else {
      odometry.enableVisionPoseRotation();
    }
  }
}
