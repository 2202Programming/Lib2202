// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.util;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.Constants;
import frc.lib2202.command.WatcherCmd;
@SuppressWarnings("rawtypes")
public class MaxServo implements VelocityControlled {
    String name = "no-name";
    Class mtrClass;
    // commands
    
    double trim = 0.0; // offset from the commanded position (not seen in measured)
    double MIN_POS = -500.0, MAX_POS = 500.0; // PLEASE SET YOUR CLAMP VALUES
   
    // measured values
    double currentPos;
    double currentVel;

    double setPoint;

    // safety checks on servo movement
    int safety_frame_count = 0;
    final int WARNING_MSG_FRAMES = 250;  // every 5 seconds, warn about stall
    int warning_count = 0; 
    
    int NO_MOTION_FRAMES = 25;           // 0.5 seconds
    double stall_limit_current = 60.0;   //track output current limit for stall protection
    int prev_direction = 0;             // 0 no stall, otherwise sign of velocity

    // state vars
    final public PIDFController hwVelPIDcfg; // matches hardware setting
    final ClosedLoopSlot hwVelSlot;

    // hardware
    final SparkBase ctrl;
    final SparkBaseConfig ctrlCfg;
    final SparkClosedLoopController pid;
    final RelativeEncoder encoder;
    RelativeEncoder posEncoder = null;

    /*
     * internal constructor shared by other forms, does most of setup
     * encoder and pid set in calling constructor.
     */

    private MaxServo(int canID, MotorType motorType,
            PIDFController hwVelPIDcfg,
            boolean inverted, ClosedLoopSlot hwVelSlot,
            Type encType, int kCPR, Class mtrClass, double maxVelocity, double maxAccel, double allowedError) {
        this.mtrClass = mtrClass;
        ctrl = (mtrClass == SparkMax.class) ? new SparkMax(canID, motorType) : new SparkFlex(canID, motorType);
        ctrlCfg = (mtrClass == SparkMax.class) ? new SparkMaxConfig() : new SparkFlexConfig();
        setName("MaxServo-" + canID);  //until a better name is selected
        ctrl.setCANTimeout(50); //enter blocking mode for config
        ctrl.clearFaults();
        //not used libs2025 update  ctrl.restoreFactoryDefaults();
        ctrlCfg.inverted(inverted)
              .idleMode(IdleMode.kBrake);
        ctrlCfg.closedLoop.maxMotion
              .maxVelocity(maxVelocity)
              .maxAcceleration(maxAccel)
              .allowedClosedLoopError(allowedError);

        if (encType == null) {
            //normal, internal encoder based on motor counts                      
            ctrlCfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
                .outputRange(-1.0, 1.0);                      
        }
        else if (encType == Type.kQuadrature) {
            ctrlCfg.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .outputRange(-1.0, 1.0);
            
            // dpl 1/4/2025 looks like only kQuadrature is only type supported.
            if(mtrClass ==  SparkMax.class){
                ((SparkMaxConfig) ctrlCfg).alternateEncoder
                .countsPerRevolution(kCPR)
                .inverted(false);
            } else {
                ((SparkFlexConfig) ctrlCfg).externalEncoder
                .countsPerRevolution(kCPR)
                .inverted(false);
            }
                /*******************  may need these 
                .averageDepth(tbd)
                .positionConversionFactor(tbd)
                .velocityConversionFactor(tbd)
                .setSparkMaxDataPortConfig()
                ***********************************/
        }
        else {
            System.out.println("MAX SERVO config error ... STOPPING with NPE");
            throw new NullPointerException();
        }
       
        // copy rest of inputs
        this.hwVelSlot = hwVelSlot;
        this.hwVelPIDcfg = hwVelPIDcfg;
        
        //Setup pid on hardware with give config and full output range
        this.hwVelPIDcfg.copyTo(ctrl, ctrlCfg, this.hwVelSlot); 
        
         // apply ctrlCfg
        ctrl.configure(ctrlCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
         // now that ctrl is fully spec'd get support devices
        posEncoder = encoder = ctrl.getEncoder(); 
        pid = ctrl.getClosedLoopController(); 

        errorCheck();
        ctrl.setCANTimeout(0); //leave blocking mode
    }

   

    void errorCheck() {
        REVLibError error = ctrl.getLastError();
        if (error!= REVLibError.kOk) {
            System.out.println(name + " SparkMax/Flex LastError:" + error.toString() + ", clearing." );
        }
        error = ctrl.clearFaults();
        if (error!= REVLibError.kOk) {
            System.out.println(name + " SparkMax/Flex couldn't clear faults:" + error.toString() + ", good luck." );
        }
    }

    public MaxServo setName(String name) {
        this.name = name;
        return this;
    }

    /*
     * Add an external Position encoder.  Trying to use motor counts for velocity and external 
     * encoder for position.
     * 
     * encType - Type.kQuadrature
     * CPR - count / rotation likely 8192
     * scale_rotation - [units/rotation]
     */
    public MaxServo addAltPositionEncoder(Type encType, int CPR, double scale_rotations){
        if(mtrClass == SparkMax.class){
            ((SparkMaxConfig) ctrlCfg).alternateEncoder
            .countsPerRevolution(CPR)
            .positionConversionFactor(scale_rotations)
            .velocityConversionFactor( scale_rotations / 60.0);
        } else {
            ((SparkFlexConfig) ctrlCfg).externalEncoder
            .countsPerRevolution(CPR)
            .positionConversionFactor(scale_rotations)
            .velocityConversionFactor( scale_rotations / 60.0);
        }
        ctrl.configure(ctrlCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return this;
    }

    // SmartMax Neo specific tuning methods

    /**
     * Set motor's conversion factor.
     * This is typically used to accomodate gearboxes and to allow control in "Engineered Units". 
     * The position control will return rotations multiplied by the supplied conversion factor
     * The velocity control will return rotations per second multiplied by the supplied conversion factor.
     * 
     * position will be in engineered units like:  [deg] or [cm]
     * velocity will be in [deg/s] or [cm/s]
     * 
     * For example a motor connected to a 45:1 reduction gearbox which rotates an arm in which the desired 
     * control is degs/second should supply: (1/45)[rot-out]/rot-mtr](360 [deg/rot-out]) as the conversion factor.
     * 
     *      convFactor + 360.0/45.0 = 8.0 [deg/rot-mtr]
     * 
     * Always leave the conversion factor unsimplified to avoid magic numbers and easy editing if one part
     * of the mechanism changes.
     * 
     * This is for the default internal encoder used in spareMax/flux. If an alternate or external encoder
     * is needed use addAltPositionEncoder()  
     * @see MaxServo.addAltPositionEncoder(Type encType, int CPR, double scale_rotations
     * 
     * @see com.revrobotics.spark.config.EncoderConfig#positionConversionFactor(double factor)
     * @see <a href="https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/encoderconfig#positionConversionFactor(double)"></a>
     * @see com.revrobotics.spark.config.EncoderConfig#velocityConversionFactor(double factor)
     * @see <a href="https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/encoderconfig#velocityConversionFactor(double)"></a>
     * 
     * @param conversionFactor A double representing the conversion factor
     * @return The modified MaxServo object for method chaining
     */
    public MaxServo setConversionFactor(double conversionFactor) {
        //use the normal internal encoder
        EncoderConfig enc = (mtrClass == SparkMax.class) ?
                    ((SparkMaxConfig)ctrlCfg).encoder  :
                    ((SparkFlexConfig)ctrlCfg).encoder ;
        enc .positionConversionFactor(conversionFactor)
            .velocityConversionFactor( conversionFactor / 60.0);
        ctrl.configure(ctrlCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return this;
    }


    public MaxServo setSmartCurrentLimit(int stallLimit, int freeLimit) {
        // sets curren limits, but RPMLimit is not enabled, 10K is default
        return setSmartCurrentLimit(stallLimit, freeLimit, 10000);
    }

    public MaxServo setSmartCurrentLimit(int stallLimit, int freeLimit, int rpmLimit) {
        // save stall current limit derated for stall check
        this.stall_limit_current = stallLimit*0.95;
        ctrlCfg.smartCurrentLimit(stallLimit, freeLimit, rpmLimit);
        ctrl.configure(ctrlCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return this;
    }

    public MaxServo setVelocityHW_PID(double smVelMax, double smAccelMax) {
        // write the hwVelPIDcfgcfg constants to the sparkmax
        hwVelPIDcfg.copyTo(ctrl, ctrlCfg, hwVelSlot, smVelMax, smAccelMax);
        return this;
    }
   
    public SparkBase getController() {
        return ctrl;
    }

    public MaxServo setBrakeMode(IdleMode mode) {        
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(mode);
        ctrl.configureAsync(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        return this;
    }

    // Servo's position setpoint
    public void setSetpoint(double pos) {
        pos = MathUtil.clamp(pos, MIN_POS, MAX_POS);
        setPoint=pos;
    }

    public void setClamp(double min_pos, double max_pos) {
        MIN_POS = min_pos;
        MAX_POS = max_pos;
    }

   
    public double getPosition() {
        return currentPos;
    }

    public void setTrim(double trim) {
        this.trim = trim;
    }

    public double getTrim() {
        return trim;
    }
    
    //TODO: is this still needed
    public void clearHwPID() {
        ctrl.getClosedLoopController().setIAccum(0.0);
    }

    /*
     * isStalled()
     * 
     * Looks for motion on the servo to ensure we are not stalled.
     * 
     * return -
     *  false => servo is moving correctly
     *  true => servo is stalled for N frames or more, cut the motor in periodic()
     */
    //TODO: reimplement
    boolean isStalled(){
        return false;
    }
       


    public void periodic() {
        periodic(0.0);
    }

    /*
     * compAdjustment - used to sync two servo like arms that should move together
     */
    public void periodic(double compAdjustment) {
        // measure -read encoder for current position and velocity
        currentPos = posEncoder.getPosition() - trim;
        currentVel = encoder.getVelocity();

        // confirm we are moving and not stalled
        if (isStalled()) {
            // issue stall warning, but not every frame
            if ((warning_count++ % WARNING_MSG_FRAMES) == 0) {
                DriverStation.reportError(name + " servo stalled at\n" +
                    "  pos          =" + currentPos + "\n" +
                    "  set point    =" + positionPID.getSetpoint() + "\n" +
                    "  velocity_cmd =" + velocity_cmd + "\n" +
                    "  measured_vel =" + currentVel +  "\n" +
                    "  current      =" + ctrl.getOutputCurrent() + "\n" +
                    "  appliedOutput=" + ctrl.getAppliedOutput() + "\n\n", false);
            }
            // stalled for NO_MOTION_FRAMES frames, stop trying to move                    
        }
        else {
            // moving, clear the warning counter
            warning_count = 0;
        }
        // potential use of feedforward
        pid.setReference(setPoint, ControlType.kMAXMotionPositionControl, hwVelSlot, ArbFFUnits.kPercentOut);
    }

    public Command getWatcher() {
        return new MaxWatcher();
    }
//TODO: update with relevant MAX servo values
    class MaxWatcher extends WatcherCmd {
        NetworkTableEntry nt_currentPos;
        NetworkTableEntry nt_desiredPos;
        NetworkTableEntry nt_maxVelocity;
        NetworkTableEntry nt_maxAccel;
        NetworkTableEntry nt_allowedError;
        NetworkTableEntry nt_trim;

        @Override
        public String getTableName() {
            return name; // from MaxServo
        }

        @Override
        public void ntcreate() {
            NetworkTable table = getTable();
            nt_arbFF = table.getEntry("ArbFF");
            nt_currentPos = table.getEntry("Position");
            nt_maxVelocity = table.getEntry("MaxVelocity");
            nt_desiredPos = table.getEntry("PositionCmd");
            nt_allowedError = table.getEntry("allowedError");
            nt_maxAccel = table.getEntry("maxAccel");
            nt_trim = table.getEntry("Trim");

            // put the a copy on dashboard to edit
            SmartDashboard.putData(name + "/hwVelPIDcfg", hwVelPIDcfg);
        }

        @Override
        public void ntupdate() {
        
            nt_currentPos.setDouble(getPosition());
            nt_maxVelocity.setDouble(getMaxVel());
            nt_desiredPos.setDouble(getSetpoint());
            nt_maxAccel.setDouble(getMaxAccel());
            nt_allowedError.setDouble(getAllowedError());
            nt_trim.setDouble(trim);

            // look for PIDF config changes
            hwVelPIDcfg.copyChangesTo(ctrl, ctrlCfg, hwVelSlot);
        }
    }

}