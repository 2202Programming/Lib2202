package frc.lib2202.util;

import static frc.lib2202.Constants.DT;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * PIDFController - extends current (2020) pidcontroller to include a feed
 * forward gain which is not currently part of the WPILib version.
 * 
 * This is useful for holding values for devices like the talon SRX or sparkMax
 * which may have a feed forward gain or Izone.
 * 
 * 2/16/21 added CopyTo helper functions
 * 
 */
public class PIDFController extends PIDController {
    // hardware refs if used, set by first copyTo()
    SparkBase hw_controller = null;
    SparkBaseConfig hw_config = null;

    double m_smartMaxVel = 0.1;
    double m_smartMaxAccel = .01;
    String m_name = "";          // can't be final, but NT setup deferred until we have a name
    double m_Kf = 0.0;
    
    private Boolean NT_enabled = false;
    private NetworkTable table;
    private NetworkTableEntry nt_p;
    private NetworkTableEntry nt_i;
    private NetworkTableEntry nt_d;
    private NetworkTableEntry nt_f;

    private NetworkTableEntry nt_requested_p;
    private NetworkTableEntry nt_requested_i;
    private NetworkTableEntry nt_requested_d;
    private NetworkTableEntry nt_requested_f;

    public final String NT_Name = "PIDF"; // expose data under PIDF table

    /**
     * Construct a PIDF controller given the following gains.
     * 
     * When tuning a velocity PID, start with P, I and D all as 0 and f as 1/conversionFactor. 
     * Then slowing increase f and setpoint until you have reasonable movement. 
     * Adjust f such that when movement occurs, it is happening at the rate specified in engineered 
     * units in the setpoint. Confirm this with glass or anothe plotting method
     * 
     * When tuning position PID, start with I and D at 0 and P at a small (0.1) value.
     * Increase P until you get small oscillations, and set P to 0.6*value with oscillations.
     * Optionally, add a small I if mechanism is having trouble getting to final position.
     * 
     * @param Kp proportional gain 
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param Kf Feed-Forward gain
     * 
     * @see edu.wpi.first.math.controller.PIDController#PIDController(double kp, double ki, double kd, double period)
     * @see <a href="https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning">Getting Started With PID Tuning</a>
     */
    public PIDFController(double Kp, double Ki, double Kd, double Kf) {
        this(Kp, Ki, Kd, Kf, DT,"");
    }

    /**
     * Construct a PIDF controller given the following gains.
     * 
     * @see {@link #PIDFController(double Kp, double Ki, double Kd, double Kf, double period)}
     * 
     * @param Kp proportional gain 
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param Kf Feed-Forward gain
     * @param period Default controller update rate
     * 
     */
    public PIDFController(double Kp, double Ki, double Kd, double Kf, double period) {
        this(Kp, Ki, Kd, Kf, period, "");
    }

    /**
     * Construct a PIDF controller with the same Kp, Ki, Kd, Kf, and period as the supplied
     * controller
     * @param src The controller to source the PID gains from
     * 
     * @see {@link #PIDFController(double Kp, double Ki, double Kd, double Kf, double period)}
     */
    // public PIDFController(PIDFController src) {
    //     this(src.getP(), src.getI(), src.getD(), src.getF(), src.getPeriod(), src.m_name+"_copied");
    // }

    /**
     * Construct a PIDF controller with a name for network tables for tuning
     * @param m_name String for PIDFController NT entries
     * 
     * @see {@link #PIDFController(double Kp, double Ki, double Kd, double Kf)}
     */
    public PIDFController(double Kp, double Ki, double Kd, double Kf, String m_name) {
        this(Kp, Ki, Kd, Kf, DT, m_name);
    }

    /**
     * Construct a PIDF controller with a name for network tables for tuning
     * @param m_name String for PIDFController NT entries
     * 
     * @see {@link #PIDFController(double Kp, double Ki, double Kd, double Kf, double period)}
     */
    public PIDFController(double Kp, double Ki, double Kd, double Kf, double period, String m_name) {
        super(Kp, Ki, Kd, period);
        setF(Kf);
        setName(m_name);       
    }

    /**
     * @see {@link #PIDFController(double Kp, double Ki, double Kd, double Kf)}
     */
    public void setPIDF(double kP, double kI, double kD, double kF) {
        setPID(kP, kI, kD);
        setF(kF);
    }

    // Accessors for the Kf
    public double getF() {
        return m_Kf;
    }

    public void setF(double Kf) {
        m_Kf = Kf;
    }

    public String getName(){
        return m_name;
    }
    
    public void setName(String m_name) {
        //skip if we don't have a name yet
        if (m_name.equals("")) return;
        
        // allow name change only of NT wasn't setup
        if (this.m_name.equals("")) {
            this.m_name = m_name;
            // initialize NT
            NT_setup();           
        }
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint    The new setpoint of the controller.
     */
    @Override
    public double calculate(double measurement, double setpoint) {
        return super.calculate(measurement, setpoint) + (m_Kf * setpoint);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     */
    @Override
    public double calculate(double measurement) {
        return calculate(measurement, getSetpoint());
    }

    /**
     * Copied from base class and feed forward added.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("f", this::getF, this::setF);
        builder.addDoubleProperty("iZone", this::getIZone, this::setIZone);
    }

    public boolean equals(PIDFController other) {
        return getP() == other.getP() && getI() == other.getI() && 
              getD() == other.getD() && getF() == other.getF();
    }

    /**
     * 
     * copyTo() copies this pid's values down to a hardward PID implementation
     * 
     * @param motorController  device to change
     * @param motorConfig      device's config object
     * @param slot          control slot on device
     * 
     *                      optional smartMax vel and accel limits may be given
     * @param smartMaxVel   optional, 0.1 [units/s]
     * @param smartMaxAccel optional 0.01 [units/s^2]
     */
    public void copyTo(SparkBase motorController, SparkBaseConfig motorConfig, ClosedLoopSlot slot) {
        copyTo(motorController, motorConfig, slot, m_smartMaxVel, m_smartMaxAccel);
    }

    public void copyTo(SparkBase motorController, SparkBaseConfig motorConfig) {
        copyTo(motorController, motorConfig, ClosedLoopSlot.kSlot0, m_smartMaxVel, m_smartMaxAccel);
    }

    public void copyTo(SparkBase motorController, SparkBaseConfig motorConfig, ClosedLoopSlot slot, 
                       double smartMaxVel, double smartMaxAccel) {
        m_smartMaxVel = smartMaxVel;
        m_smartMaxAccel = smartMaxAccel;
       
        //need to check - if we just update a few parameters in SparkMaxConfig, do the rest stay the same as previously set?
        //otherwise do we need to pull all the prior parameters out of the motorController's sparkmaxconfig and reapply them?
        motorConfig.closedLoop.pidf(this.getP(), this.getI(), this.getD(), this.getF(), slot);
        // sparkmax does not like POSITIVE_INFINITY, thows param erron on id 17...  0.0 should workaround
        // minor hack: if izone is not set, base class defaults to POSITIVITE_INFINITY
        double izone = (this.getIZone() == Double.POSITIVE_INFINITY) ? 0.0 : this.getIZone();
        motorConfig.closedLoop.iZone(izone, slot);
        motorConfig.closedLoop.maxMotion.maxAcceleration(smartMaxAccel, slot);
        motorConfig.closedLoop.maxMotion.maxVelocity(smartMaxVel, slot);

        REVLibError driveError = motorController.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        //save hw references for later use in 
        if (hw_controller == null) hw_controller = motorController;
        if (hw_config == null) hw_config = motorConfig;

        if(driveError != REVLibError.kOk)
            System.out.println("*** ERROR *** SparkMax Flash Failed during copyTo command. Error val=" + driveError);
    }

    public void copyChangesTo(SparkBase controller, SparkBaseConfig motorConfig) {
        copyChangesTo(controller, motorConfig, ClosedLoopSlot.kSlot0);
    }

    // compares an updated PIDF with this one and updates it and the hardware
    public void copyChangesTo(SparkBase motorController, SparkBaseConfig motorConfig, ClosedLoopSlot slot) {
        boolean changed = false;

        // skip if no hw, typical if use PIDF without calling copyTo()
        if (motorConfig == null || motorController == null) return;

        var pidCfg =  motorConfig.closedLoop;

        //accessor lets us get the pid values in the motorController
        ClosedLoopConfigAccessor pid_acc = (motorController instanceof SparkMax) ?
            ((SparkMax)motorController).configAccessor.closedLoop :
            ((SparkFlex)motorController).configAccessor.closedLoop;
        
        // compare PIDF values with acc, update pidCfg values that have changed
        if (compare(getP() ,pid_acc.getP()) ) {            
            pidCfg.p(getP(), slot);
            changed = true;
        }

        if (compare(getI(), pid_acc.getI())) {
            pidCfg.i(getI(), slot);
            changed = true;
        }

        if (compare(getD(), pid_acc.getD())) {
            pidCfg.d(getD(), slot);
            changed = true;
        }

        if (compare(getF(), pid_acc.getFF())) {      
            pidCfg.velocityFF(getF(), slot);
            changed = true;
        }

        if (compare(getIZone(), pid_acc.getIZone())) {           
            pidCfg.iZone(getIZone(), slot);
            changed = true;
        }
        // send to HW if we have a pid change, use async so robot loop isn't delayed
        if (changed) {
            motorController.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);                
        }
    }

    /*
     * doubles between pidf and the controller won't match exactly
     * so limit the number of bits compared on the diff.
     */
    boolean compare(double x1, Double x2) {
        double diff = Math.abs(x1 - x2)*100000.0; // 5 sig digits
        return diff > 0.01;
    }
    //TODO - add back for the CTRE controllers (find in older repo)


    /**
     * NT_setup()
     * 
     * Set up NetworkTables for PIDF gain settings. This is a debug feature to allow adjustment of
     * gains without rebuilding and redeploying code. Enabled if the name param is set
     */
    private void NT_setup(){
        table = NetworkTableInstance.getDefault().getTable(NT_Name);

        // Check if entry already exists
        while(table.containsSubTable(m_name)){
            System.err.println("NetworkTable SubTable already exists for key `" + m_name + "`.");
            System.err.println("!!Rename one of the " + m_name + " PIDF Objects!!");
            m_name = m_name + "-duplicate";
            System.err.println("Renaming to `" + m_name + "`");
        }
        table = table.getSubTable(m_name);

        nt_p = table.getEntry("/Current P");
        nt_i = table.getEntry("/Current I");
        nt_d = table.getEntry("/Current D");
        nt_f = table.getEntry("/Current F");

        //set initial requested values to be current PIDF values
        nt_p.setDouble(getP());
        nt_i.setDouble(getI());
        nt_d.setDouble(getD());
        nt_f.setDouble(getF());

        // Setup requested entries
        nt_requested_p = table.getEntry("/Requested P");
        nt_requested_p.setDouble(getP());
        nt_requested_i = table.getEntry("/Requested I");
        nt_requested_i.setDouble(getI());
        nt_requested_d = table.getEntry("/Requested D");
        nt_requested_d.setDouble(getD());
        nt_requested_f = table.getEntry("/Requested F");
        nt_requested_f.setDouble(getF());
        
        NT_enabled = true;
    }

    public void NT_update() {
        // skip if not setup
        if (!NT_enabled) return;

        // Update readout values
        nt_p.setDouble(getP());
        nt_i.setDouble(getI());
        nt_d.setDouble(getD());
        nt_f.setDouble(getF());

        // check if requested values are different from current values, update if needed
        boolean updatePID = false;
        // Validate P
        if (nt_requested_p.getDouble(-1) < 0.0) {
            System.err.print("Invalid P value. Must be a positive double\n");
        }else if (nt_requested_p.getDouble(-1) != getP()){
            updatePID = true;
        }
        // Validate I
        if(nt_requested_i.getDouble(-1) < 0.0) {
            System.err.print("Invalid I value. Must be a positive double\n");
        }else if (nt_requested_i.getDouble(-1) != getI()){
            updatePID = true;
        }
        // Validate D
        if(nt_requested_d.getDouble(-1) < 0.0) {
            System.err.print("Invalid D value. Must be a positive double\n");
        }else if (nt_requested_d.getDouble(-1) != getD()){
            updatePID = true;
        }
        // Validate F
        if(nt_requested_f.getDouble(-1) < 0.0) {
            System.err.print("Invalid FF value. Must be a double\n");
        }else if (nt_requested_f.getDouble(Double.MIN_VALUE) != getF()){
            updatePID = true;
        }

        if (updatePID){
            System.out.println(m_name + ": Updating PIDF values to requested values");
            setPIDF(nt_requested_p.getDouble(getP()), 
                    nt_requested_i.getDouble(getI()), 
                    nt_requested_d.getDouble(getD()), 
                    nt_requested_f.getDouble(getF()));
            
            // copy values to hw
            copyChangesTo(hw_controller, hw_config);
        }
    }
}
