package frc.lib2202.command.swerve;

import static frc.lib2202.Constants.DEGperRAD;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib2202.util.ModMath;

public class StrafeDrive extends FieldCentricDrive {

  final static double PIDAngleRange = 180.0;
  final static double KP = 5.0; // [[deg/s] / deg-error]
  final static double VyMin = 0.2; // [m/s] min vy to allow rotation

  final double facing_offset; // desired rotation offset for strafing
  final double intake_heading; // robot coords of intake, typically 0 degrees
  final PIDController rotationPID; // drives our facing

  boolean is_blue;
  double target_facing;
  double rb_facing;

  public StrafeDrive(double facing_offset) {
    this(facing_offset, 0.0); //most of time intake is at 0 deg heading
  }

  public StrafeDrive(double facing_offset, double intake_heading) {
    super();
    this.facing_offset = facing_offset;
    this.intake_heading = intake_heading;
    rotationPID = new PIDController(KP, 0.0, 0.0);
    rotationPID.enableContinuousInput(-PIDAngleRange, PIDAngleRange);
  }

  @Override
  public void initialize() {
    is_blue = DriverStation.getAlliance().get().equals(Alliance.Blue);
    rb_facing = is_blue ? intake_heading + 180.0 : intake_heading;
    rotationPID.reset();
    SmartDashboard.putString("TargetCentricDrive/State", "StrafeDrive");
  }


  double calculate_rotation(double Vy) {
    currrentHeading = gyro.getHeading();
    target_facing = currrentHeading.getDegrees();  // don't move if Vy is small
    // if moving, move to our target rotation point.
    if (Vy > VyMin) { 
      target_facing = -facing_offset;
      target_facing = ModMath.fmod360(rb_facing + target_facing);
    } else if (Vy < -VyMin) {
      target_facing = facing_offset;
      target_facing = ModMath.fmod360(rb_facing + target_facing);
    }
   
    // use our target and current heading to command a rotation
    double rot_cmd = rotationPID.calculate(currrentHeading.getDegrees(), target_facing);
    return rot_cmd / DEGperRAD;  //rotation cmd must be in radians
  }

  @Override
   void calculate() {
    //We override the super's calculate(), so we can replace the rotation
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * limits.kMaxSpeed;
    ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * limits.kMaxSpeed;
    double rot_cmd = calculate_rotation(ySpeed);

    // Clamp speeds/rot from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -limits.kMaxSpeed, limits.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -limits.kMaxSpeed, limits.kMaxSpeed);
    rot = MathUtil.clamp(rot_cmd, -limits.kMaxAngularSpeed, limits.kMaxAngularSpeed);

    //convert field centric speeds to robot centric
    ChassisSpeeds tempChassisSpeed = (DriverStation.getAlliance().get().equals(Alliance.Blue)) 
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading) 
        : ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, rot, currrentHeading); // if on red alliance you're looking at robot from opposite. Pose is in blue coordinates so flip if red

    output_states = kinematics.toSwerveModuleStates(tempChassisSpeed);
  }

  // public void execute()  - using base class is fine

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }


  // allow pid parameters to change
  public StrafeDrive setP(double kp) {
    this.rotationPID.setP(kp);
    return this;
  }

  public StrafeDrive setI(double ki) {
    this.rotationPID.setI(ki);
    return this;
  }

  public StrafeDrive setD(double kd) {
    this.rotationPID.setD(kd);
    return this;
  }

}
