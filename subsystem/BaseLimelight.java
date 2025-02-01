package frc.lib2202.subsystem;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.Constants;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.LimelightHelpers.LimelightTarget_Fiducial;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;

public abstract class BaseLimelight extends SubsystemBase {

    protected NetworkTable table;
    protected NetworkTable outputTable;

    protected NetworkTableEntry leds;
    protected NetworkTableEntry booleanLeds;
    protected NetworkTableEntry NT_hasTarget;
    protected NetworkTableEntry nt_bluepose_x;
    protected NetworkTableEntry nt_bluepose_y;
    protected NetworkTableEntry outputTx;
    protected NetworkTableEntry outputTv;
    protected NetworkTableEntry pipelineNTE;
    protected NetworkTableEntry nt_numApriltags;
    protected NetworkTableEntry distanceToTargetTag;

    protected double x;
    protected double filteredX;
    protected double y;
    protected double area; // area is between 0 and 100. Calculated as a percentage of image
    protected boolean targetValid;
    protected boolean ledStatus; // true = ON
    protected double filteredArea;

    // botpose index keys
    static final int X = 0;
    static final int Y = 1;
    static final int Z = 2;
    static final int RX = 3;
    static final int RY = 4;
    static final int RZ = 5;

    protected long pipeline;

    @SuppressWarnings("unused")
    protected LinearFilter x_iir;
    @SuppressWarnings("unused")
    protected LinearFilter area_iir;
    public final String NT_Name = "Vision"; // expose data under DriveTrain table
    protected double filterTC = 0.08; // seconds, 2Hz cutoff T = 1/(2pi*f) was .2hz T=.8
    protected int log_counter = 0;

    // generalize April Tag Target
    protected int targetID = -1;
    protected Translation2d targetTag = null;

    // private Pose2d megaPose;
    protected Pose2d teamPose = new Pose2d(); // todo hack inits to avoid NPE 4/8/2023
    protected Pose2d bluePose = new Pose2d();
    protected int numAprilTags;
    protected double visionTimestamp;

    protected String name;

    public BaseLimelight(String limelight) {
        this.name = limelight;
        x_iir = LinearFilter.singlePoleIIR(filterTC, Constants.DT);
        area_iir = LinearFilter.singlePoleIIR(filterTC, Constants.DT);
        table = NetworkTableInstance.getDefault().getTable(this.name);  // was "limelight"); //TODO test with multiple Limelights
        outputTable = NetworkTableInstance.getDefault().getTable(NT_Name + this.name);

        leds = table.getEntry("ledMode");
        booleanLeds = table.getEntry("booleanLeds");
        pipelineNTE = table.getEntry("pipeline");

        // these are "output" entries for user debugging
        nt_bluepose_x = outputTable.getEntry("/LL Blue Pose X");
        nt_bluepose_y = outputTable.getEntry("/LL Blue Pose Y");
        nt_numApriltags = outputTable.getEntry("/LL_Num_Apriltag");
        NT_hasTarget = outputTable.getEntry("/LL hasTarget");
        outputTv = outputTable.getEntry("/Limelight Valid");
        outputTx = outputTable.getEntry("/Limelight X error");
        distanceToTargetTag = outputTable.getEntry("/Distance To TargetTag");
        disableLED();

    }

    public String getName(){ return this.name;}

    /*
     * LL pose estimate may need a starting point to work from.
     * This matches PhotonVision and will be called anytime
     * the drivetrain's pose is reset. See swerverDrivetrain.java.
     */
    public void setInitialPose(Pose2d pose, double time) {
        // TODO - What is needed here? Even used?
        System.out.println("LL_setInitialPose():" + pose.toString() + " function needs implementation...");
    }

    public void setTarget(int id, Translation2d location ) {
        targetID = id;
        targetTag = location;
    }

    public double getVisionTimestamp() {
        return visionTimestamp;
    }

    public Pose2d getBluePose() {
        return bluePose;
    }

    public Pose2d getTeamPose() {
        return teamPose;
    }

    public int getNumApriltags() {
        return numAprilTags;
    }

    public boolean hasAprilTarget() {
        return getNumApriltags() > 0;
    }

    public double getX() {
        return x;
    }

    public double getFilteredX() {
        return filteredX;
    }

    public double[] getAprilTagID() {
        LimelightHelpers.LimelightTarget_Fiducial[] apriltag = LimelightHelpers.getLatestResults(name).targets_Fiducials;
        double[] tagIDs = new double[apriltag.length];
        for (int i = 0; i < apriltag.length; i++) {
            tagIDs[i] = apriltag[i].fiducialID;
        }
        return tagIDs;
    }

    public double getTA() {
        return LimelightHelpers.getTA("");
    }

    public LimelightTarget_Fiducial[] getAprilTagsFromHelper(){
        return LimelightHelpers.getLatestResults(name).targets_Fiducials;
    }

    public double getFilteredArea() {
        return filteredArea;
    }

    public double getY() {
        return y;
    }

    public double getArea() {
        return area;
    }

    public boolean getTarget() {
        return targetValid;
    }

    public boolean getLEDStatus() {
        return ledStatus;
    }

    public void disableLED() {
        leds.setNumber(1);
        ledStatus = false;
        booleanLeds.setBoolean(ledStatus);
    }

    public void enableLED() {
        leds.setNumber(3);
        ledStatus = true;
        booleanLeds.setBoolean(ledStatus);
    }

    public void toggleLED() {
        if (ledStatus) {
            disableLED();
        } else {
            enableLED();
        }

    }

    public void setPipeline(int pipe) {
        LimelightHelpers.setPipelineIndex(this.name, pipe);
        if (pipe == 1) {
            enableLED();
        } else
            disableLED();
    }

    // switch between pipeline 0 and 1
    public void togglePipeline() {
        long pipe = pipelineNTE.getInteger(0);
        if (pipe == 0) {
            setPipeline(1);
            pipeline = 1;
        } else {
            setPipeline(0);
            pipeline = 0;
        }
    }

    public long getPipeline() {
        return pipeline;
    }

    public boolean valid() {
        return targetValid;  //TODO doesn't seem like this was set right -dpl
    }

    public void log() {
        log_counter++;
        if (log_counter % 20 == 0) {
            NT_hasTarget.setBoolean(targetValid);

            if (bluePose != null) {
                nt_bluepose_x.setDouble(bluePose.getX());
                nt_bluepose_y.setDouble(bluePose.getY());
            }

            outputTv.setValue(targetValid);
            outputTx.setDouble(x);

            if (targetTag != null && targetID > 0) {
                distanceToTargetTag
                        .setDouble(((DriveTrainInterface)RobotContainer.getSubsystem("drivetrain")).getDistanceToTranslation(targetTag));
            }
        }
    }

}
