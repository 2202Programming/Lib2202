// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.subsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class Limelight extends BaseLimelight {
 // BaseLimelight contains working vars and API accessors
  public Limelight(){
    this("limelight");
  }

  // in case we have multiple limelights, use different name, include name in tables
  public Limelight(String name) {
    super(name);
  }

  @Override
  public void periodic() {
    pipeline = pipelineNTE.getInteger(0);

    // LL apriltags stuff
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(this.name);
    numAprilTags = llresults.targets_Fiducials.length;
  
    visionTimestamp = Timer.getFPGATimestamp() 
        - (LimelightHelpers.getLatency_Pipeline(this.name) / 1000.0)
        - (LimelightHelpers.getLatency_Capture(this.name) / 1000.0);

    if (numAprilTags > 0) {
      bluePose = LimelightHelpers.getBotPose2d_wpiBlue(this.name);
      teamPose = bluePose;  //LimelightHelpers.getBotPose2d_wpiBlue(this.name); // assume/default blue for now

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == Alliance.Red)
        // aliance info exists AND is red
        teamPose = LimelightHelpers.getBotPose2d_wpiRed(this.name);
    }
    
    // update a few NT entriesS every frame
    nt_numApriltags.setInteger(numAprilTags);
    log(); // do logging at end
  }

}
