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
public class NeoServo implements VelocityControlled {
    String name = "no-name";
    Class mtrClass;
    // commands
    double velocity_cmd; // computed from pid, or external_vel_cmd
    double maxVelocity; // limits
    double initialMaxVelocity = 0.0; // keep the first non-zero as the hard max
    double arbFeedforward = 0.0; // for specialized control cases
    double external_vel_cmd = 0.0; // for velocity_mode == true
    boolean velocity_mode = false;
    double trim = 0.0; // offset from the commanded position (not seen in measured)
    double MIN_POS = -500.0, MAX_POS = 500.0; // PLEASE SET YOUR CLAMP VALUES
   
    // measured values
    double currentPos;
    double currentVel;

    // safety checks on servo movement
    int safety_frame_count = 0;
    final int WARNING_MSG_FRAMES = 250;  // every 5 seconds, warn about stall
    int warning_count = 0; 
    
    int NO_MOTION_FRAMES = 25;           // 0.5 seconds
    double stall_limit_current = 60.0;   //track output current limit for stall protection
    int prev_direction = 0;             // 0 no stall, otherwise sign of velocity

    // state vars
    final PIDController positionPID;
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

    private NeoServo(int canID, MotorType motorType,
            PIDController positionPID,
            PIDFController hwVelPIDcfg,
            boolean inverted, ClosedLoopSlot hwVelSlot,
            Type encType, int kCPR, Class mtrClass) {
        this.mtrClass = mtrClass;
        ctrl = (mtrClass == SparkMax.class) ? new SparkMax(canID, motorType) : new SparkFlex(canID, motorType);
        ctrlCfg = (mtrClass == SparkMax.class) ? new SparkMaxConfig() : new SparkFlexConfig();
        setName("NeoServo-" + canID);  //until a better name is selected
        ctrl.setCANTimeout(50); //enter blocking mode for config
        ctrl.clearFaults();
        //not used libs2025 update  ctrl.restoreFactoryDefaults();
        ctrlCfg.inverted(inverted)
               .idleMode(IdleMode.kBrake);
       
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
            System.out.println("NEO SERVO config error ... STOPPING with NPE");
            throw new NullPointerException();
        }
       
        // copy rest of inputs
        this.positionPID = positionPID;
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

    /* default slot0 for pid slot */
    public NeoServo(int canID, PIDController positionPID, PIDFController hwVelPIDcfg, boolean inverted) {
        this(canID, positionPID, hwVelPIDcfg, inverted, ClosedLoopSlot.kSlot0);
    }

    public NeoServo(int canID, PIDController positionPID, PIDFController hwVelPIDcfg, boolean inverted, ClosedLoopSlot hwVelSlot) {
        this(canID, MotorType.kBrushless, positionPID, hwVelPIDcfg, inverted,  hwVelSlot, 
            null, 0, SparkMax.class);        
    }

    // Works now with given motor type and an alt encoder
    public NeoServo(int canID, MotorType motorType,
            PIDController positionPID,
            PIDFController hwVelPIDcfg,
            Type extEncoderType, int kCPR,  
            boolean inverted, ClosedLoopSlot hwVelSlot) {
        this(canID, MotorType.kBrushless, positionPID, hwVelPIDcfg, inverted,  hwVelSlot, 
            extEncoderType, kCPR, SparkMax.class);        
    }

    public NeoServo(int canID, PIDController positionPID, PIDFController hwVelPIDcfg, boolean inverted, Class mtrClass) {
        this(canID, positionPID, hwVelPIDcfg, inverted, ClosedLoopSlot.kSlot0, mtrClass);
    }

    public NeoServo(int canID, PIDController positionPID, PIDFController hwVelPIDcfg, boolean inverted, ClosedLoopSlot hwVelSlot, Class mtrClass) {
        this(canID, MotorType.kBrushless, positionPID, hwVelPIDcfg, inverted,  hwVelSlot, 
            null, 0, mtrClass);        
    }

    // Works now with given motor type and an alt encoder
    public NeoServo(int canID, MotorType motorType,
            PIDController positionPID,
            PIDFController hwVelPIDcfg,
            Type extEncoderType, int kCPR,  
            boolean inverted, ClosedLoopSlot hwVelSlot, Class mtrClass) {
        this(canID, MotorType.kBrushless, positionPID, hwVelPIDcfg, inverted,  hwVelSlot, 
            extEncoderType, kCPR, mtrClass);        
    }

    void errorCheck() {
        REVLibError error = ctrl.getLastError();
        if (error!= REVLibError.kOk) {
            System.out.println(name + " SparkMax LastError:" + error.toString() + ", clearing." );
        }
        error = ctrl.clearFaults();
        if (error!= REVLibError.kOk) {
            System.out.println(name + " SparkMax couldn't clear faults:" + error.toString() + ", good luck." );
        }
    }

    public NeoServo setName(String name) {
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
    public NeoServo addAltPositionEncoder(Type encType, int CPR, double scale_rotations){
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
     * @see NeoServo.addAltPositionEncoder(Type encType, int CPR, double scale_rotations
     * 
     * @see com.revrobotics.spark.config.EncoderConfig#positionConversionFactor(double factor)
     * @see <a href="https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/encoderconfig#positionConversionFactor(double)"></a>
     * @see com.revrobotics.spark.config.EncoderConfig#velocityConversionFactor(double factor)
     * @see <a href="https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/encoderconfig#velocityConversionFactor(double)"></a>
     * 
     * @param conversionFactor A double representing the conversion factor
     * @return The modified NeoServo object for method chaining
     */
    public NeoServo setConversionFactor(double conversionFactor) {
        //use the normal internal encoder
        EncoderConfig enc = (mtrClass == SparkMax.class) ?
                    ((SparkMaxConfig)ctrlCfg).encoder  :
                    ((SparkFlexConfig)ctrlCfg).encoder ;
        enc .positionConversionFactor(conversionFactor)
            .velocityConversionFactor( conversionFactor / 60.0);
        ctrl.configure(ctrlCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return this;
    }

    /**
     * Set the motor's positional and velocity tolerance. If SetConversionFactor() was called, 
     * this will be in engineering units
     * 
     * @param posTol motor's positional tolerance in engineering units
     * @param velTol motor's velocity tolerance in engineering units
     * @return The modified NeoServo object for method chaining
     */
    public NeoServo setTolerance(double posTol, double velTol) {
        positionPID.setTolerance(posTol, velTol);
        return this;
    }

    public NeoServo setSmartCurrentLimit(int stallLimit, int freeLimit) {
        // sets curren limits, but RPMLimit is not enabled, 10K is default
        return setSmartCurrentLimit(stallLimit, freeLimit, 10000);
    }

    public NeoServo setSmartCurrentLimit(int stallLimit, int freeLimit, int rpmLimit) {
        // save stall current limit derated for stall check
        this.stall_limit_current = stallLimit*0.95;
        ctrlCfg.smartCurrentLimit(stallLimit, freeLimit, rpmLimit);
        ctrl.configure(ctrlCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return this;
    }

    public NeoServo setVelocityHW_PID(double smVelMax, double smAccelMax) {
        // write the hwVelPIDcfgcfg constants to the sparkmax
        hwVelPIDcfg.copyTo(ctrl, ctrlCfg, hwVelSlot, smVelMax, smAccelMax);
        return this;
    }

    public NeoServo burnFlash() {
        // TODO - clean up burnflash option, now its embedded in configure() call
        //ctrl.burnFlash();
        //Timer.delay(.2); // this holds up the current thread
        return this;
    }

    /**
     * Sets the motor controller's max velocity in both directions.
     * Units are in engineering units if setConversionFactor() was called 
     * @param maxVelocity maximum motor velocity in engineering units
     * @return this NeoServo to facilitate chaining
     */
    public NeoServo setMaxVelocity(double maxVelocity) {
        // defers to setMaxVel() VelocityControlled API, but returns this for config chaining
        setMaxVel(maxVelocity);
        return this;
    }

    public SparkBase getController() {
        return ctrl;
    }

    public NeoServo setBrakeMode(IdleMode mode) {        
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(mode);
        ctrl.configureAsync(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        return this;
    }

    /*
     * VelocityControlled API
     * 
     */
    // Servo's position setpoint
    public void setSetpoint(double pos) {
        pos = MathUtil.clamp(pos, MIN_POS, MAX_POS);
        positionPID.setSetpoint(pos);
        velocity_mode = false;
        external_vel_cmd = 0.0;
    }

    public void setClamp(double min_pos, double max_pos) {
        MIN_POS = min_pos;
        MAX_POS = max_pos;
    }

    public boolean isVelocityMode() {
        return velocity_mode;
    }

    public double getSetpoint() {
        return positionPID.getSetpoint();
    }

    public boolean atSetpoint() {
        return positionPID.atSetpoint();
    }

    // Sets the encoder position (Doesn't move anything)
    public void setPosition(double pos) {
        posEncoder.setPosition(pos); // tell our encoder we are at pos
        positionPID.reset(); // clear any history in the pid
        positionPID.calculate(pos - trim, pos); // tell our pid we want that position; measured, setpoint same
    }

    public double getPosition() {
        return currentPos;
    }

    public void setMaxVel(double v) {
        v = Math.abs(v);
        if (initialMaxVelocity == 0.0)
            initialMaxVelocity = v; // saving the initial as hard max
        maxVelocity = (v <= initialMaxVelocity) ? v : initialMaxVelocity;
    }

    public double getMaxVel() {
        return maxVelocity;
    }

    public void setVelocityCmd(double vel) {
        velocity_mode = true;
        external_vel_cmd = vel;
    }

    public double getVelocity() {
        return currentVel;
    }

    public double getVelocityCmd() {
        return velocity_cmd;
    }

    public void setArbFeedforward(double aff) {
        // uses percent power, on range of -1. to 1.0
        if (Math.abs(aff) > 1.0) {
            DriverStation.reportError("|ArbFF| > 1, check your math. Using ZERO.", true);
            arbFeedforward = 0.0;
        } else
            arbFeedforward = aff;
    }

    public void setTrim(double trim) {
        this.trim = trim;
    }

    public double getTrim() {
        return trim;
    }

    public void hold() {
        external_vel_cmd = 0.0;
        currentPos = posEncoder.getPosition();

        // set our setpoint, but stay in whatever control mode is being used
        positionPID.calculate(currentPos, currentPos);
    }

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
    boolean isStalled() {
        int direction = (int)Math.copySign(1.0, velocity_cmd);
        // check stall conditions
        boolean not_moving = 
                (Math.abs(velocity_cmd) > positionPID.getErrorDerivativeTolerance()) && // motion requested
                (Math.abs(currentVel) < positionPID.getErrorDerivativeTolerance()) &&   // motion not seen
                (!positionPID.atSetpoint()) &&          // not at our goal
                (prev_direction == direction) &&        // still trying to move in same direction
                (ctrl.getOutputCurrent() * ctrl.getAppliedOutput() > stall_limit_current) && // drawing high current                          
                (DriverStation.isEnabled());            // is enabled
        
        safety_frame_count = not_moving ? ++safety_frame_count : 0;
        prev_direction = direction;
        return safety_frame_count > NO_MOTION_FRAMES;
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

        // velocity_mode, update position setpoint so we don't jump back on mode switch
        if (velocity_mode) {
            // 4/10/2023 dpl positionPID.reset();
            positionPID.setSetpoint(currentPos);
        }

        // calculate - run position pid to get velocity
        velocity_cmd = MathUtil.clamp(positionPID.calculate(currentPos) + compAdjustment, -maxVelocity, maxVelocity);

        // if velocity mode, use external_vel_cmd, otherwise use positionPID
        velocity_cmd = velocity_mode ? external_vel_cmd + compAdjustment : velocity_cmd;

        // local copy of arbFeedforward incase stall has to zero it and it isn't set
        // every frame by class user
        double arbFF = arbFeedforward;

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
            velocity_cmd = 0.0;
            arbFF = 0.0;             
        }
        else {
            // moving, clear the warning counter
            warning_count = 0;
        }
        // potential use of feedforward
        pid.setReference(velocity_cmd, ControlType.kVelocity, hwVelSlot, arbFF, ArbFFUnits.kPercentOut);
    }

    public void simulationPeriodic() {
        // nothing to do if we are not enabled
        if (!DriverStation.isEnabled())
            return;
        // simple model - encoder vel is set in sim when a velocity mode is used
        // so move the position based on velocity being commanded
        // no dynamics are modeled
        double pos = posEncoder.getPosition() + encoder.getVelocity() * Constants.DT;
        posEncoder.setPosition(pos);
    }

    public Command getWatcher() {
        return new NeoWatcher();
    }

    class NeoWatcher extends WatcherCmd {
        NetworkTableEntry nt_arbFF;
        NetworkTableEntry nt_currentPos;
        NetworkTableEntry nt_desiredPos;
        NetworkTableEntry nt_desiredVel;
        NetworkTableEntry nt_currentVel;
        NetworkTableEntry nt_trim;

        @Override
        public String getTableName() {
            return name; // from NeoServo
        }

        @Override
        public void ntcreate() {
            NetworkTable table = getTable();
            nt_arbFF = table.getEntry("ArbFF");
            nt_currentPos = table.getEntry("Position");
            nt_currentVel = table.getEntry("Velocity");
            nt_desiredPos = table.getEntry("PositionCmd");
            nt_desiredVel = table.getEntry("VelocityCmd");
            nt_trim = table.getEntry("Trim");

            // put the a copy on dashboard to edit
            SmartDashboard.putData(name + "/hwVelPIDcfg", hwVelPIDcfg);
        }

        @Override
        public void ntupdate() {
            nt_arbFF.setDouble(arbFeedforward);
            nt_currentPos.setDouble(getPosition());
            nt_currentVel.setDouble(getVelocity());
            nt_desiredPos.setDouble(getSetpoint());
            nt_desiredVel.setDouble(getVelocityCmd());
            nt_trim.setDouble(trim);

            // look for PIDF config changes
            hwVelPIDcfg.copyChangesTo(ctrl, ctrlCfg, hwVelSlot);
        }
    }

}