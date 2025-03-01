package frc.lib2202.command.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.subsystem.OdometryInterface;

public class MoveToPoint extends Command {
  final static double RampTime = 1.5; //[1/s]
  
  // sdt and odo/gyro are set in the AutoBuilder
  //DriveTrainInterface sdt;
  //IHeadingProvider gyro;
  //final GoalEndState endState;


  OdometryInterface odo;
  Pose2d targetPose;
  Command pathfindingCommand;
  
  final String odoName;
  final PathConstraints constraints;
  
  /**
   * Constructs a MoveToPoint
   * Uses default constraints for velocity and accelerations.
   * Expects the OdometryInterface SS to be named "odometry".
   * @param targetPose target pose2D
   */
  public MoveToPoint(Pose2d targetPose) {
    this("odometry", getConstraints(),targetPose );
  }

   /**
   * Constructs a MoveToPoint
   * @param odoName name of odometry SS to use
   * @param targetPose target pose2D
   */
  public MoveToPoint(String odoName, Pose2d targetPose) {
    this(odoName, getConstraints(),targetPose );
  }
  /**
   * Constructs a MoveToPoint
   * @param odoName name of odometry SS to use
   * @param constraints path constraints
   * @param targetPose target pose2D
   */
  public MoveToPoint(String odoName, PathConstraints constraints, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.odoName = odoName;
    this.constraints = (constraints != null) ? constraints : getConstraints();
    this.targetPose = targetPose;
   
    //sdt = RobotContainer.getSubsystem("drivetrain");
    //gyro = RobotContainer.getRobotSpecs().getHeadingProvider();
    odo = RobotContainer.getSubsystemOrNull(odoName);
    
    // endstate has zero velocity, and our endpoint's rotation
    //endState = new GoalEndState(0.0, targetPose.getRotation());
  }

  //use the RobotSpecs' RobotLimits, and RampTime above to create path constaints
  static PathConstraints getConstraints() {
    RobotLimits limits = RobotContainer.getRobotSpecs().getRobotLimits();
    var c = new PathConstraints(limits.kMaxSpeed, limits.kMaxSpeed * RampTime,
                limits.kMaxAngularSpeed, limits.kMaxAngularSpeed*RampTime);
    return c;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (odo == null) {
      System.out.println("Odometry " + odoName + " not found, not running path.");
      return;
    }

    if (!AutoBuilder.isConfigured()) {
      System.out.println("AutoBuilder not configured, not running path.");
      return;
    }

    //compute path to point, run it.
    pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints,0.0);  
    pathfindingCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      pathfindingCommand.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathfindingCommand.isFinished();
  }

 
}
