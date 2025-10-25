package frc.lib2202.command.pathing;

import java.util.LinkedHashSet;
import java.util.function.Consumer;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AlianceAwareSetPose extends InstantCommand {
    // keep a list of callbacks because different estimators/gyros need to 
    // know about changes.
    // called in order the callbacks are added.
    static LinkedHashSet<Consumer<Pose2d>> callBacks = new LinkedHashSet<Consumer<Pose2d>>();
    static LinkedHashSet<Consumer<Rotation2d>> rotCallBacks = new LinkedHashSet<Consumer<Rotation2d>>();

    // set of callbacks for systems that need to know when gyro reset is needed
    public static void  AddPose2dCallback(Consumer<Pose2d> pose2d_consumer) {        
        callBacks.add(pose2d_consumer);
    }
    public static void AddRotationCallback(Consumer<Rotation2d> rot2d_consumer) {
        rotCallBacks.add(rot2d_consumer);
    }

    //starting pose on blue side
    final Pose2d pose; // blue, gets transposed to red if needed

    protected AlianceAwareSetPose() {
        // null pose - here to support sub-classes that create Pose2d and override initialize()
        this(null, (Consumer<Pose2d>[]) null);
    }

    public AlianceAwareSetPose(Pose2d poseBlue) {
        this(poseBlue, (Consumer<Pose2d>[]) null);
    }

    @SuppressWarnings("unchecked")
    public AlianceAwareSetPose(Pose2d poseBlue, Consumer<Pose2d>... consumers) {
        this.pose = poseBlue;
        if (consumers != null) {
            for (Consumer<Pose2d> consumer : consumers) {
                AddPose2dCallback(consumer);
            }
        }
    }

    @Override
    public void initialize() {
        Pose2d alliancePose;
        //protect from null pose
        if (pose == null) {
            return;
        }
        // red needs to flip, static internal to FlippingUtil determines mirror or rotation
        alliancePose = (DriverStation.getAlliance().get() == Alliance.Blue) ?  pose 
                            : FlippingUtil.flipFieldPose(pose);     
        call_consumers(alliancePose);
        // print what we were set too
        DriverStation.reportWarning("*** AWSetPose() " + alliancePose.toString(), false);
    }

    public static void call_consumers(Pose2d pose){
        if (pose == null) return;

        // check each part because a rotation only may null the translation object.
        if (pose.getTranslation() != null) {
            // call everyone that cares about pose2d reset
            for (Consumer<Pose2d> consumer : callBacks) {
                consumer.accept(pose);
            }
        }
        if (pose.getRotation() != null) {
            // call everyone that cares about rotation2d, ignore translation2d, like gyros
            var rot2d = pose.getRotation();
            for (Consumer<Rotation2d> consumer : rotCallBacks) {
                consumer.accept(rot2d);
            }
        }
    }
    
    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
}