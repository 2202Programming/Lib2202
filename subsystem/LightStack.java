package frc.lib2202.subsystem;

/*
The LightStack class is designed to manage a stack of lights, allowing for easy control and manipulation of multiple lights in a coordinated manner. This addition will enhance the functionality of our lighting system and provide a more efficient way to handle multiple light sources in our projects.
of CANdle lights. The LightStack class provides methods to requst a LightRequest, the Light Request contains a priority that is used to prioritize all of the LIghtRequests and only display the one with the hioghest priority.
LightStack is a singleton and must be registered, it can be called outside of commands as though it were a logger.

THere are 2 COnstructors for LightStack, one that takes an array of CANdleIDs / Defaults to 8 lights, and one that takes arrays of CANdleIDs and Length of light strip. and a default LightRequest. The default LightRequest is used when there are no other LightRequests in the stack, it is the default state of the lights.

It is designed to make it easier to create complex lighting patterns and effects. 

Example usage:

Register like BlinkyLights with CANdle CAN Ids

            .add(LightStack.class, "LIGHTS", () -> {
                return new LightStack(CAN.CANDLE1, CAN.CANDLE2);
            })

Request a light pattern from anywhere in the code like this:

LightStack.request(LightRequest.getCritical());

or

LightStack.request(LightRequest.getTeamRed())   );

or 

Lightstack.request(LightRequest.getDefault().color(LightRequest.ORANGE).animation(AnimationType.FADE, 1.0).setSeconds(5.0));

or 

LightStack.request(LightRequest.geColor(255, 0, 255).animation(AnimationType.RAINBOW, 0.2).setSeconds(10.0));

Many of the animations ignore the color, but for those that use it, you can specify any RGB color you want. You can also specify a duration for the request, after which it will expire and be removed from the stack. If multiple requests are active at the same time, the one with the highest priority will be shown on the lights. If two requests have the same priority, the most recently added one will take precedence.

*/

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; - Do we do check voltage somewhere else in our code? If so, we can remove Smart dashboard
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class LightStack extends SubsystemBase {

    public enum AnimationType {
        SOLID, BLINK, FADE, RAINBOW, FIRE, STROBE
    }

    public static class LightRequest {
        public static final Color8Bit BLACK = new Color8Bit(0, 0, 0), WHITE = new Color8Bit(255, 255, 255),
                RED = new Color8Bit(255, 0, 0), GREEN = new Color8Bit(0, 255, 0),
                BLUE = new Color8Bit(0, 0, 255), ORANGE = new Color8Bit(255, 145, 0),
                YELLOW = new Color8Bit(255, 255, 0);

        public static final int PRIORITY_LOW = 10, PRIORITY_NORMAL = 100, PRIORITY_HIGH = 1000,
                PRIORITY_VERYHIGH = 10000, PRIORITY_CRITICAL = 1000000;

        private int priority = PRIORITY_NORMAL;
        private Color8Bit color = WHITE;
        private int startLed = 0;
        private int length = 8;
        private int candleIndex = 0;
        private AnimationType animationType = AnimationType.SOLID;
        private double periodSeconds = 0.5;
        private double timeRemaining = Double.MAX_VALUE;

        private LightRequest() {
        }

        // --- Static Factory Methods ---
        public static LightRequest getDefault() {
            return new LightRequest();
        }

        public static LightRequest getWhite() {
            return new LightRequest().color(WHITE);
        }

        public static LightRequest getRed() {
            return new LightRequest().color(RED);
        }

        public static LightRequest getBlue() {
            return new LightRequest().color(BLUE);
        }

        public static LightRequest getTeamRed() {
            return new LightRequest().setPriority(PRIORITY_LOW).color(RED).setSeconds(Double.MAX_VALUE); // will always fall back to team colors if nothing else is going on
        }

        public static LightRequest getTeamBlue() {
            return new LightRequest().setPriority(PRIORITY_LOW).color(BLUE).setSeconds(Double.MAX_VALUE);// will always fall back to team colors if nothing else is going on
        }

        public static LightRequest getCritical() {
            return new LightRequest().setPriority(PRIORITY_CRITICAL).animation(AnimationType.STROBE, 0.1).color(RED);
        }

        // --- Fluent Modifiers ---
        public LightRequest setPriority(int priority) {
            this.priority = priority;
            return this;
        }

        public LightRequest color(Color8Bit c) {
            this.color = c;
            return this;
        }

        public LightRequest color(int r, int g, int b) {
            return color(new Color8Bit(r, g, b));
        }

        public LightRequest setSeconds(double duration) {
            this.timeRemaining = duration;
            return this;
        }

        /** Default animation speed of 0.5 seconds */
        public LightRequest animation(AnimationType type) {
            return animation(type, 0.5);
        }

        public LightRequest animation(AnimationType type, double speedInSeconds) {
            this.animationType = type;
            this.periodSeconds = speedInSeconds;
            return this;
        }

        public LightRequest onDevice(int index) {
            this.candleIndex = index;
            return this;
        }

        public LightRequest range(int start, int numLeds) {
            this.startLed = start;
            this.length = numLeds;
            return this;
        }

        // used for decrementing light request times, should be called by LightStack
        // only
        private void updateTime(double deltaTime) {
            if (this.timeRemaining != Double.MAX_VALUE) {
                this.timeRemaining -= deltaTime;
            }
        }

        // used for checking expired light requests, should be called by LightStack only
        private boolean isFinished() {
            return this.timeRemaining <= 0;
        }

        // too complex to overrie equals and hashcode, just do a manual compare when we
        // need it
        public boolean isSameAs(LightRequest other) {
            if (other == null)
                return false;
            return this.priority == other.priority &&
                    this.color.equals(other.color) &&
                    this.animationType == other.animationType &&
                    this.periodSeconds == other.periodSeconds &&
                    this.startLed == other.startLed;
        }
    }

    private static class Device {
        public final CANdle hardware;
        public final int ledCount; // default to 8, but can be set by constructor
        public LightRequest currentlyShowing = null;

        public Device(int id, int count) {
            this.hardware = new CANdle(id, "rio");
            this.ledCount = count;
        }

        /** Overloaded constructor if we just want the default 8 */
        // ledCount is 8 be default, this is the "base" number of lights on a CANdle, if
        // we add more use the other constructor
        // if some day we want to split the longer strips into separate "devices" we'll
        // want another constructor and rewrite all the device code to support multi
        // commands but setions of lights :P
        public Device(int id) {
            this(id, 8); // This calls the constructor above
        }

    }

    private final List<Device> candleList = new ArrayList<>();
    private final List<TreeMap<Integer, LightRequest>> stackList = new ArrayList<>();
    private double lastTime;
    private static LightStack instance;

    public LightStack(int[][] candleIDandLength) {
        if (instance != null) {
            throw new IllegalStateException("LightStack constructor called twice!");
        }

        for (int[] config : candleIDandLength) {
            candleList.add(new Device(config[0], config[1]));
            stackList.add(new TreeMap<>());
        }
        this.lastTime = Timer.getFPGATimestamp();

        instance = this; // singleton instance set to this object
    }

    public LightStack(int[] candleIds) {
        if (instance != null) {
            throw new IllegalStateException("LightStack constructor called twice!");
        }

        for (int id : candleIds) {
            candleList.add(new Device(id));
            stackList.add(new TreeMap<>());
        }
        this.lastTime = Timer.getFPGATimestamp();

        instance = this; // singleton instance set to this object
    }

    /**
     * Gets the LightStack singleton.
     * 
     * @throws IllegalStateException if called before the constructor has run.
     */
    public static LightStack getInstance() {
        if (instance == null) {
            // DataLogManager allows the error to be captured in the 'Black Box'
            DataLogManager.log("CRITICAL: LightStack accessed before initialization!");

            throw new IllegalStateException(
                    "LightStack singleton accessed before it was initialized in RobotContainer. " +
                            "Check your construction order!");
        }
        return instance;
    }

    public static void request(LightRequest req) {

        if (req.candleIndex < instance.stackList.size()) {
            instance.stackList.get(req.candleIndex).put(req.priority, req);
        }
    }

    @Override
    public void periodic() {
        double deltaTime = Timer.getFPGATimestamp() - lastTime;
        lastTime = Timer.getFPGATimestamp();

        for (int i = 0; i < candleList.size(); i++) {
            Device device = candleList.get(i);
            TreeMap<Integer, LightRequest> stack = stackList.get(i);

            // Brownout check - Do we do this somewhere else in our code? If so, we can
            // remove Smart dashboard
            // Brownout - Shut diwn the lights and save power for the robot to keep moving,
            // is < 7.0 volts to low should it be < 8.0 volts?

            double vbat = device.hardware.getSupplyVoltage().getValueAsDouble();
            // SmartDashboard.putNumber("LED/VBat_" + i, vbat);
            if (vbat < 7.0) {
                // clear animation to turn off LEDs
                device.hardware.setControl(new com.ctre.phoenix6.controls.NeutralOut());
                continue;
            }

            // Tick and Clean
            List<Integer> expiredPriorities = new ArrayList<>();
            for (Map.Entry<Integer, LightRequest> entry : stack.entrySet()) {
                entry.getValue().updateTime(deltaTime);
                if (entry.getValue().isFinished())
                    expiredPriorities.add(entry.getKey());
            }
            for (Integer p : expiredPriorities)
                stack.remove(p);

            LightRequest winner = stack.isEmpty() ? null : stack.lastEntry().getValue();

            // Hardware Sync
            if (winner == null) {
                if (device.currentlyShowing != null) {
                    // clear animation to turn off LEDs
                    device.hardware.setControl(new com.ctre.phoenix6.controls.NeutralOut());
                    device.currentlyShowing = null;
                }
            } else if (!winner.isSameAs(device.currentlyShowing)) {
                applyToCANdle(device.hardware, winner);
                device.currentlyShowing = winner;
            }
        }
    }

    private void applyToCANdle(CANdle candle, LightRequest req) {
        // 1. Phoenix 6 uses a NeutralOut control to clear the device
        candle.setControl(new NeutralOut());

        // 2. Prepare the color using the RGBWColor class from your example
        // Phoenix 6 colors are typically handled via RgbwColor or Color
        var targetColor = new RGBWColor(req.color.red, req.color.green, req.color.blue);

        // 3. Switch based on animation type using the with... modifiers
        switch (req.animationType) {
            case BLINK:
            case STROBE:
                // Constructor: StrobeAnimation(startLed, numLed)
                candle.setControl(new StrobeAnimation(req.startLed, req.length)
                        .withColor(targetColor)
                        .withFrameRate(1.0 / Math.max(req.periodSeconds, 0.001)));
                break;

            case FADE:
                // Constructor: SingleFadeAnimation(startLed, numLed)
                candle.setControl(new SingleFadeAnimation(req.startLed, req.length)
                        .withColor(targetColor)
                        .withFrameRate(1.0 / Math.max(req.periodSeconds, 0.001)));
                break;

            case RAINBOW:
                // Constructor: RainbowAnimation(startLed, numLed)
                // Speed for Rainbow is a scalar (higher is faster)
                candle.setControl(new RainbowAnimation(req.startLed, req.length)
                        .withFrameRate(req.periodSeconds));
                break;

            case FIRE:
                // Constructor: FireAnimation(startLed, numLed)
                candle.setControl(new FireAnimation(req.startLed, req.length)
                        .withFrameRate(req.periodSeconds));
                break;

            case SOLID:
            default:
                // Use the direct pixel setter for solid colors
                candle.setControl(new SolidColor(req.startLed, req.length).withColor(targetColor));
                break;
        }
    }
}