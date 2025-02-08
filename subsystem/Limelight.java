// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.subsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class Limelight extends BaseLimelight {
  /** Creates a new Limelight_Subsystem. */

  public Limelight(){
    this("limelight");
  }

  // in case we have multiple limelights, use different name, include name in tables
  public Limelight(String name) {
    super(name);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();

    pipeline = pipelineNTE.getInteger(0);

    // LL apriltags stuff
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(this.name);
    numAprilTags = llresults.targets_Fiducials.length;
    nt_numApriltags.setInteger(numAprilTags);
    visionTimestamp = Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline(this.name) / 1000.0)
        - (LimelightHelpers.getLatency_Capture(this.name) / 1000.0);

    if (numAprilTags > 0) {
      bluePose = LimelightHelpers.getBotPose2d_wpiBlue(this.name);
      teamPose = LimelightHelpers.getBotPose2d_wpiBlue(this.name); // assume/default blue for now

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == Alliance.Red)
        // aliance info exists AND is red
        teamPose = LimelightHelpers.getBotPose2d_wpiRed(this.name);
    }
  }

}
