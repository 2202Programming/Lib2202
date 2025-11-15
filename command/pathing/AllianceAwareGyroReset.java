package frc.lib2202.command.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceAwareGyroReset extends AlianceAwareSetPose {  
  static final Pose2d Heading0 =  new Pose2d(null,  Rotation2d.fromDegrees(0.0));
  static final Pose2d Heading180 = new Pose2d(null, Rotation2d.fromDegrees(180.0));

  // Shares static callbacks in AlianceAwareSetPose to update subscribers
  // Subsystems can call the static Add<Pose2d|Rotation2d>Callback() to ensure they are updated.

  //resets the robot rotation/gyro, assuming robot is facing AWAY from driver, and will use alliance
  //to determine if that means 0 or 180 degrees.
  public AllianceAwareGyroReset() {}
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d rot_only = (DriverStation.getAlliance().get() == Alliance.Blue) ? Heading0 : Heading180;    
    call_consumers(rot_only);
  }
}
