// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.subsystem;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.Optional;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.builder.RobotContainer;

public class BlinkyLights extends SubsystemBase {
    // Some common m_colors
    static public Color8Bit BLACK = new Color8Bit(0, 0, 0);
    static public Color8Bit WHITE = new Color8Bit(255, 255, 255);
    static public Color8Bit RED = new Color8Bit(255, 0, 0);
    static public Color8Bit GREEN = new Color8Bit(0, 255, 0);
    static public Color8Bit BLUE = new Color8Bit(0, 0, 255);
    static public Color8Bit ORANGE = new Color8Bit(255, 145, 0);
    static public Color8Bit YELLOW = new Color8Bit(255, 255, 0);

    static final ArrayList<Color8Bit> EXAMPLE_INDIV_COLORS = new ArrayList<>() 
        {{
            add(WHITE); // Candle 0
            add(ORANGE); // Candle 1 ...            
        }};

    public interface IColorProvider {
        // override with your color functions as desired
        default public Color8Bit colorProvider() {
            return null;
        };

        default public ArrayList<Color8Bit> individualColorProvider() {
            // overide and return array if you want to set different m_colors to each candle.
            // most of the time you don't need this
            return null;
        };

        default public boolean requestBlink() {
            return false;
        }

        default public double requestBrightness(){
            return 1.0;
        }

        // robot mode changes
        default public void onRobotInit() { };
        default public void onAutomousInit() { };
        default public void onTeleopInit() { };
        default public void onTestInit() { };
    }

    // variables and constants
    static final int TO = 50; // [ms] timeout for comms
    // candle frame update rate, larger saves on CAN BUS
    static final int FrameStatusTime = 200; // [ms]

    // removed static to support multiple uses on different light groups
    private BlinkyLightUser m_currentUser = null;

    // State vars, handle N candle sets
    ArrayList<CANdle> m_candles = new ArrayList<CANdle>();
    ArrayList<Color8Bit> m_colors = new ArrayList<Color8Bit>();   // current colors in candle order
    double m_bightness;
    boolean m_blinkstate;

    final BlinkyLightUser  m_defaultUser;
    final LinkedHashSet<BlinkyLightUser> m_users = new LinkedHashSet<>();

    public BlinkyLights(int... can_ids) {       
         m_defaultUser = new BlinkyLightUser(this) {
            @Override
            public Color8Bit colorProvider() {  return null;
            }
        };
        // initialize devices
        for (int id : can_ids) {
            var candle = new CANdle(id);
            config(candle);
            m_candles.add(candle);
            m_colors.add(BLACK);
        }
        
        m_currentUser =  m_defaultUser;
    }

    // blinkylights config
    void config(CANdle cdl) {
        final var StatusFrame = CANdleStatusFrame.CANdleStatusFrame_Status_1_General;
        var cfg = new CANdleConfiguration();
        cdl.clearStickyFaults(TO);
        cfg.enableOptimizations = true;
        cdl.configAllSettings(cfg, TO);

        // lower CAN bus usage for CANdle
        int period = cdl.getStatusFramePeriod(StatusFrame);
        if (period < FrameStatusTime)
            cdl.setStatusFramePeriod(StatusFrame, FrameStatusTime, TO);

        cdl.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, FrameStatusTime);
        cdl.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, FrameStatusTime);
    }

    void AddUser(BlinkyLightUser user){
        m_users.add(user);
    }

    void updateSettings(IColorProvider cp) {
        // read values from cp
        Color8Bit newColor = cp.colorProvider();
        ArrayList<Color8Bit> newIndividualColors = cp.individualColorProvider();
        boolean newBlink = cp.requestBlink();
        double brightness = cp.requestBrightness();

        // avoid CAN bus traffic if color isn't changing
        for (int i = 0; i < m_candles.size(); i++) {
            Color8Bit color = null;
            if (newIndividualColors != null && i < newIndividualColors.size()) {
                color = newIndividualColors.get(i);
            } else {
                // no entry in list, use single color
                color = newColor;
            }
            // update if different
            if (color != null && m_colors.get(i) != color)
                setIndividualColor(i, color);
            // blink & brightness
            if (m_blinkstate != newBlink)
                setBlinking(i, newBlink, m_colors.get(i));
            if  (m_bightness != brightness)
                setBrightness(i, brightness);        
        }
        // update the state for grouped behavoirs
        m_blinkstate = newBlink;
        m_bightness = brightness;
    }


    //gives up control of lights, switch to next user or default.
    public void release() {
        // find first User Cmd that is scheduled
        for (BlinkyLightUser u : m_users ){
            boolean running = (u.watchCmd != null && u.isScheduled() && u.active);
            if (!running) continue;

            //found someone that wants to run
            m_currentUser = u;
            return;
        }
        m_currentUser =  m_defaultUser;
    }

    /* Static methods to intercept robot state changes */
    public void onRobotInit() {
        m_currentUser.onRobotInit();
    };

    public void onDisabledPeriodic() {
        // always do our alliance m_colors
        setAllianceColors();
    }

    public void onAutomousdInit() {
        m_currentUser.onAutomousInit();
    };

    public void onTeleopInit() {
        m_currentUser.onTeleopInit();
    };

    public void onTestInit() {
        m_currentUser.onTestInit();
    };

    public void onTestPeriodic() {
        m_currentUser.onTestInit();
    };

    // can be used directly from SS
    public void setColor(Color8Bit color) {
        for (int i = 0; i < m_candles.size(); i++) {
           setIndividualColor(i, color);
        }
    }

    public void setIndividualColor(int idx, Color8Bit color) {
        if (idx < 0 || idx > m_candles.size() - 1) return;
        if (!differentColors(m_colors.get(idx), color)) return;
        // different color, update it          
        m_candles.get(idx).setLEDs(color.red, color.green, color.blue);
        m_colors.set(idx, color);    
        
    }

    public void setBlinking(int idx, boolean blink, Color8Bit color) {
        if (idx < 0 || idx > m_candles.size() - 1) return;
        CANdle c = m_candles.get(idx);
        if (blink) {
            Animation animation = new StrobeAnimation(color.red, color.green, color.blue, 0, 0.5, 8);           
            c.animate(animation, 0);
        }
        else {            
            c.clearAnimation(0);
        }
    }

    /*
     * Brightness on a scale from 0-1, with 1 being max brightness
     */
    public void setBrightness(int idx, double brightness) {
        if (idx < 0 || idx > m_candles.size() - 1) return;
        CANdle c = m_candles.get(idx);
        c.configBrightnessScalar(brightness);        
    }

    public void setAllianceColors() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // no alliance indicated
        if (alliance.isEmpty()) {
            setColor(BLACK);
            return;
        }

        // System.out.println("***** Robot Alliance: " +
        // DriverStation.getAlliance().name());
        switch (alliance.get()) {
            case Blue:
                setColor(BLUE);
                break;
            case Red:
                setColor(RED);
                break;
            default:
                break;
        }
    }

    static boolean differentColors(Color8Bit c1, Color8Bit c2) {
        return (c1.red != c2.red) || (c1.green != c2.green) || (c1.blue != c2.blue);
    }

    /**
     * BlinkyLightUser a base command for controllering of the lights.
     * 
     * Since only one command can really
     */
    public static class BlinkyLightUser extends Command implements IColorProvider {
        final BlinkyLights lights;
        final UserWatcherCommand watchCmd;
        boolean active = false;  //true when they want the lights (ie called enable)
        final String name;

        public BlinkyLightUser() {
            this("LIGHTS");
        }

        // this version if we have multiple blinkylight sub-systems
        public BlinkyLightUser(String blinkySSName) {
            lights = RobotContainer.getObjectOrNull(blinkySSName);
            watchCmd = (lights != null) ? lights.new UserWatcherCommand(this) : null;
            name = blinkySSName + ": " + this.getClass().getSimpleName();          
        }

        //used for SubSystem's default user cmd
        BlinkyLightUser(BlinkyLights blss){
            lights = blss;
            watchCmd = null;
            name = blss.getClass().getSimpleName();
        }

        // call this to have your command take over the lights
        public void enableLights() {
            // user is a Command, setup a parallel cmd to get the color info
            if (watchCmd != null) {
                // calls updateSettings(color provider)                
                watchCmd.schedule();
                active = true;  //wants the lights
            }
        }
        // call this to give up lights early
        public void disableLights(){
            active = false;
            if (watchCmd !=null) {
                watchCmd.cancel();
                lights.release();                
            }
        }    
    } // blinkylightsUser

    // CP UserWatcherCommands are bound to a BlinkyLiight SS, so non-static
    // They periodicly call updateSettings() reading IColorProvider method
    // implemented by the parent.
    protected class UserWatcherCommand extends Command {       
        final BlinkyLightUser myUser; // a command that is using the lights
        int callCount = 0;
        
        UserWatcherCommand(BlinkyLightUser user) {
            myUser = user;
            AddUser(user);
        }

        @Override
        public void initialize() {
            callCount = 0;
        }

        /*
         * Reads providers and sends m_colors to CANDles.
         * updates done at 5hz.
         */
        @Override
        public void execute() {
            // only update lights if this is current user
            if (callCount++ % 10 == 0  && myUser == m_currentUser)
                updateSettings(myUser);
        }

        // watcher follows should always be able to
        @Override
        public boolean runsWhenDisabled() {
            return myUser.runsWhenDisabled();
        }

        @Override
        public void end(boolean interrupted) {
            myUser.disableLights();
        }

        @Override
        public boolean isFinished() {
            // run until my parent command is done
            return !myUser.isScheduled();
        }
    } //UserWatcherComand

}
