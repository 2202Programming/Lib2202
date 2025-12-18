package frc.lib2202.util;

import static frc.lib2202.Constants.DT;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.PIDController;
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
    boolean m_changes = false;   // tracks if changes need for sending to hw

    // for continous, copies in wpi are private
    double m_maximumInput;
    double m_minimumInput;

    public final String NT_Name = "PIDF"; // expose data under PIDF table

    /**
     * Construct a PIDF controller given the following gains.
     * When in doubt, use this constructor unless you need additional arguments.
     * 
     * When tuning a velocity PID, start with P, I and D all as 0 and F as <code>1/conversionFactor</code>. 
     * Then slowing increase f and setpoint until you have reasonable movement. 
     * Adjust f such that when movement occurs, it is happening at the rate specified in engineered 
     * units in the setpoint. Confirm this with glass or another plotting method.
     * 
     * When tuning position PID, start with I and D at 0 and P at a small (0.1) value.
     * Increase P until you get small oscillations, and then set P to <code>0.6*value-with-oscillations</code>.
     * Optionally, add a small I if mechanism is having trouble getting to final position.
     * 
     * @param Kp Proportional gain 
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param Kf Feed-Forward gain
     * 
     * @see {@link edu.wpi.first.math.controller.PIDController#PIDController(double kp, double ki, double kd, double period)}
     * @see <a href="https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning">Getting Started With PID Tuning</a>
     */
    public PIDFController(double Kp, double Ki, double Kd, double Kf) {
        this(Kp, Ki, Kd, Kf, DT, "");        
    }

    /**
     * Construct a PIDF controller given the following gains.
     * 
     * @see {@link #PIDFController(double Kp, double Ki, double Kd, double Kf, double period)}
     * 
     * @param Kp Proportional gain 
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
     * Construct a PIDF controller with a name for network tables for tuning
     * @param Kp Proportional gain 
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param Kf Feed-Forward gain
     * @param m_name String for PIDFController NT entries
     * 
     * @see {@link #PIDFController(double Kp, double Ki, double Kd, double Kf)}
     */
    public PIDFController(double Kp, double Ki, double Kd, double Kf, String m_name) {
        this(Kp, Ki, Kd, Kf, DT, m_name);
    }

    /**
     * Construct a PIDF controller with a name for network tables for tuning.
     * This is the base constructor, all other constructors override this with selected defaults.
     * @param Kp Proportional gain 
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param Kf Feed-Forward gain
     * @param period Default controller update rate
     * @param m_name String for PIDFController NT entries
     * 
     * @see {@link #PIDFController(double Kp, double Ki, double Kd, double Kf, double period)}
     */
    public PIDFController(double Kp, double Ki, double Kd, double Kf, double period, String m_name) {
        super(Kp, Ki, Kd, period);
        setF(Kf);
        setName(m_name);       
    }

    /* copy ctor */
    private  PIDFController(PIDFController original) {
        super(original.getP(), original.getI(), original.getD() );
        setF(original.m_Kf);
        hw_controller = original.hw_controller;
        hw_config = original.hw_config;
        m_smartMaxVel = original.m_smartMaxVel;
        m_smartMaxAccel = original.m_smartMaxAccel;
        m_minimumInput = original.m_maximumInput;
        m_maximumInput = original.m_maximumInput;
        m_name = original.m_name;
        this.setIZone(original.getIZone());
        this.setSetpoint(original.getSetpoint());
        if (original.isContinuousInputEnabled()) {
            super.enableContinuousInput(m_minimumInput, m_maximumInput);
        }        
    }

    @Override
    // captures min/max for continuous so we have it for copyCtor
    public void enableContinuousInput(double minimumInput, double maximumInput) {        
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
        super.enableContinuousInput(minimumInput, maximumInput);
      }


    /**
     * Set PIDF gain values to those given in arguments. 
     * 
     * @param Kp proportional gain 
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param Kf Feed-Forward gain
     * @param period Default controller update rate
     * @see {@link #PIDFController(double Kp, double Ki, double Kd, double Kf)}
     */
    public void setPIDF(double Kp, double Ki, double Kd, double Kf) {
        setPID(Kp, Ki, Kd);
        setF(Kf);
    }

    // Accessors for the Kf
    public double getF() {
        return m_Kf;
    }

    public void setF(double Kf) {
        m_Kf = Kf;
        m_changes = true;
    }

    @Override
    public void setP(double Kp){
        super.setP(Kp);
        m_changes = true;
    }

    @Override
    public void setI(double Ki){
        super.setI(Ki);
        m_changes = true;
    }

    @Override
    public void setD(double Kd){
        super.setD(Kd);
        m_changes = true;
    }

    @Override
    public void setIZone(double izone){
        super.setIZone(izone);
        m_changes = true;
    }

    public String getName(){
        return m_name;
    }
    
    public void setName(String m_name) {
       this.m_name = m_name;
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
        return super.calculate(measurement) + (m_Kf * getSetpoint());
    }

    /**
     * Copied from base class and feed forward added.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder); 
        builder.setSmartDashboardType(this.getClass().getSimpleName());
        builder.addDoubleProperty("f", this::getF, this::setF);
    }

    public boolean equals(PIDFController other) {
        return getP() == other.getP() && getI() == other.getI() && 
              getD() == other.getD() && getF() == other.getF();
    }

    /**
     * @see {@link #copyTo(SparkBase motorController, SparkBaseConfig motorConfig, ClosedLoopSlot slot, double smartMaxVel, double smartMaxAccel)}
     */
    public void copyTo(SparkBase motorController, SparkBaseConfig motorConfig, ClosedLoopSlot slot) {
        copyTo(motorController, motorConfig, slot, m_smartMaxVel, m_smartMaxAccel);
    }

    /**
     * @see {@link #copyTo(SparkBase motorController, SparkBaseConfig motorConfig, ClosedLoopSlot slot, double smartMaxVel, double smartMaxAccel)}
     */
    public void copyTo(SparkBase motorController, SparkBaseConfig motorConfig) {
        copyTo(motorController, motorConfig, ClosedLoopSlot.kSlot0, m_smartMaxVel, m_smartMaxAccel);
    }

    /**
     * 
     * copyTo() copies this pid's values down to a hardware PID implementation
     * 
     * @param motorController  device to change
     * @param motorConfig      device's config object
     * @param slot          control slot on device
     * 
     *                      optional smartMax vel and accel limits may be given
     * @param smartMaxVel   optional, 0.1 [units/s]
     * @param smartMaxAccel optional 0.01 [units/s^2]
     */
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
        
        //save hw references for later use in updates
        if (hw_controller == null) hw_controller = motorController;
        if (hw_config == null) hw_config = motorConfig;

        if(driveError != REVLibError.kOk)
            System.out.println("*** ERROR *** SparkMax Flash Failed during copyTo command. Error val=" + driveError);
        m_changes = false;  //hw is up to date
    }

    public void copyChangesTo(SparkBase controller, SparkBaseConfig motorConfig) {
        copyChangesTo(controller, motorConfig, ClosedLoopSlot.kSlot0);
    }

    // compares an updated PIDF with this one and updates it and the hardware
    public void copyChangesTo(SparkBase motorController, SparkBaseConfig motorConfig, ClosedLoopSlot slot) {
        // skip if no changes or no attached hw typical if use PIDF without calling copyTo()
        if (!m_changes || motorConfig == null || motorController == null) return;

        motorConfig.closedLoop.pidf(this.getP(), this.getI(), this.getD(), this.getF(), slot);        
        motorConfig.closedLoop.iZone(getIZone(), slot);
        
        // send to HW if we have a pid change, use async so robot loop isn't delayed
        motorController.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_changes = false;
    }

    public void NT_update() {
        // copy values to hw, if needed
        copyChangesTo(hw_controller, hw_config);        
    }
}
