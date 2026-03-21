package frc.lib2202.subsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib2202.subsystem.LimelightHelpers.IMUData;
import frc.lib2202.subsystem.LimelightHelpers.RawFiducial;

// fairly complete set of commads to support various LL modes,
// a composite of what we've used over past few years.
// Some simple api are implemented here, complex or stateful usage is
// handled in the subsystem implementing interface.
//
public interface ILimelight {

    public class Retro {
        public boolean tv;
        public double tx;
        public double ty;
        public double ta;
        public double txnc;
        public double tync;

        public Retro(String ll_name){
            tv = LimelightHelpers.getTV(ll_name);
            tx = LimelightHelpers.getTX(ll_name);
            ty = LimelightHelpers.getTY(ll_name);
            ta = LimelightHelpers.getTA(ll_name);
            txnc = LimelightHelpers.getTXNC(ll_name);
            tync = LimelightHelpers.getTYNC(ll_name);    
        }
    }

    public String getLLName();   // support multiple LL devices

    // field and tags
    public AprilTagFieldLayout getField();
    public void setField(AprilTagFieldLayout field);
    public int[] getTargetTags();
    public int[] setTargetTags(int[] tag_ids);
    public int[] resetTargetTags();

    // LL modes - retro or MT1/2
    public void setUseRetro(boolean use_retro);     // switches to retro mode
    public boolean getUseRetro();
    public boolean getUse_MT1();
    public void setUse_MT1(boolean use_mt1);  //switches to apriltag mode
    public boolean getUse_MT2();
    public void setUse_MT2(boolean use_mt1);

    // target bools
    public boolean getRejectUpdate();
    public boolean getTargetValid();

    // Access the MT data
    public LimelightHelpers.PoseEstimate getMt1(); //doesn't require orientation updates
    public LimelightHelpers.PoseEstimate getMt2(); // must use orientation updates, or at least 1 seed with internal IMU
    public void setRobotOrientation(Rotation2d heading);  // must call atleast once, depending on imu_mode
    
    // access reflector & pipeline
    default public void setPipeline(int pipe) {
        LimelightHelpers.setPipelineIndex(getLLName(), pipe);
    }

    default public int getPipeline(){
        return (int)LimelightHelpers.getCurrentPipelineIndex(getLLName());
    }

    public int setRetroPipeline(int new_retro_pipe);
    public int setMTPipeline(int new_mt_pipe);
    public void resetDefaultPipelines();
    

    default public void enableLED() {
        LimelightHelpers.setLEDMode_ForceOn(getLLName());
    }
    
    default public void disableLED() {
        LimelightHelpers.setLEDMode_ForceOff(getLLName());
    }

    default public boolean getLEDStatus(){
        //anything other that 1 should be some flavor of on
        return 1.0 != LimelightHelpers.getLimelightNTDouble(getLLName(), "ledMode");
    }

    public Retro getRetro();
    public boolean getRetroValid();

    // IMU for LL4
    default public void setIMUMode(int mode) {
        // LL4 modes, ignored on older LL.
        LimelightHelpers.SetIMUMode(getLLName(), mode);        
    }

    default public int getIMUMode(){
        //reads same table entry that set uses 
        return (int)LimelightHelpers.getLimelightNTDouble(getLLName(), "imumode_set");
    }
    public void setUseIMU(boolean use_imu);
    public IMUData getIMU();

    //if supported, allow switching frame processing rates.
    default public void lowPowerMode() {
        LimelightHelpers.SetThrottle(getLLName(), 200);  //skips 200 frames between updates
    }

    default public void normalPowerMode() {
        LimelightHelpers.SetThrottle(getLLName(), 0); 
    }

    //Returns the RawFiducial for the tagID or null if we don't see it.
    default public RawFiducial checkForTarget(int tagID) {
        RawFiducial[] tags = LimelightHelpers.getRawFiducials(getLLName());        
        for (RawFiducial tag : tags) {
            if (tag.id == tagID) 
                return tag;                     
        }
        return null;
    }
}        

