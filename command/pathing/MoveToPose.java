package frc.lib2202.command.pathing;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;

public class MoveToPose extends Command {
  final static double RampTimeVxy = 1.33; //[s] time to get to max vel (was 2.0)
  final static double RampTimeRot = 0.75; //[s] time to get to max rot/s

  // sdt and odo/gyro are set in the AutoBuilder
 
  //IHeadingProvider gyro;
  //final GoalEndState endState;

  final DriveTrainInterface sdt;
  final OdometryInterface odo;
  final Pose2d targetPose;
  Command pathfindingCommand;
  boolean was_scheduled;
  
  final String odoName;
  final PathConstraints constraints;
  
  /**
   * Constructs a MoveToPoint
   * Uses default constraints for velocity and accelerations from robot_spec
   * and default acceleration ramp rates
   * Expects the OdometryInterface SS to be named "odometry".
   * @param targetPose target pose2D
   */
  public MoveToPose(Pose2d targetPose) {
    this("odometry", getConstraints(RampTimeVxy, RampTimeRot), targetPose);
  }

   /**
   * Constructs a MoveToPoint
   * @param odoName name of odometry SS to use
   * @param targetPose target pose2D
   */
  public MoveToPose(String odoName, Pose2d targetPose) {
    this(odoName, getConstraints(RampTimeVxy, RampTimeRot), targetPose );
  }

   /**
   * Constructs a MoveToPoint
   * @param odoName name of odometry SS to use
   * @param targetPose target pose2D
   */
  public MoveToPose(String odoName, Pose2d targetPose, double rampTimeXY, double rampTimeRot) {
    this(odoName, getConstraints(rampTimeXY, rampTimeRot), targetPose );
  }

  /**
   * Constructs a MoveToPoint
   * @param odoName name of odometry SS to use
   * @param constraints path constraints
   * @param targetPose target pose2D
   */
  public MoveToPose(String odoName, PathConstraints constraints, Pose2d targetPose) {
    this.odoName = odoName;
    this.constraints = constraints;
    this.targetPose = targetPose;

    // odometry must be configured AND setup in AutoBuilder 
    sdt = RobotContainer.getSubsystem("drivetrain");
    odo = RobotContainer.getSubsystemOrNull(odoName);
    //because this command is going to run the calculate path I think it should have sdt requirements
    addRequirements(sdt);
  }

  //use the RobotSpecs' RobotLimits, and RampTime above to create path constaints
  static PathConstraints getConstraints(double rampTimeXY, double rampTimeRot) {
    RobotLimits limits = RobotContainer.getRobotSpecs().getRobotLimits();
    var c = new PathConstraints(limits.kMaxSpeed, limits.kMaxSpeed / rampTimeXY,
                limits.kMaxAngularSpeed, limits.kMaxAngularSpeed / rampTimeRot);
    return c;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (odo == null) {
      DriverStation.reportError("MoveToPose: Odometry " + odoName + " not found, not running path.", false);
      return;
    }

    if (!AutoBuilder.isConfigured()) {
      DriverStation.reportError("MoveToPose: AutoBuilder not configured, not running path.", false);
      return;
    }

    //compute path to point, run it.
    pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints,0.0);  

    //now run the command in this command container, don't schedule it...
    if(pathfindingCommand != null){
       pathfindingCommand.initialize();
    }
    else {
      DriverStation.reportError("MoveToPose: AutoBuilder return null command.", false);
    }
  }

  @Override
  public void execute() {
    if (pathfindingCommand == null) return;
    pathfindingCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // cleanup the pathfindingCommand
    if (pathfindingCommand != null) {
      pathfindingCommand.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var finished = (pathfindingCommand == null) || pathfindingCommand.isFinished();
    return finished;
  }
 
}
