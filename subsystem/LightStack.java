package frc.lib2202.subsystem;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * Manages a stack of light requests for CANdle LED devices with priority-based display control.
 * 
 * <p>The LightStack class is designed to manage multiple CANdle lights in a coordinated manner,
 * allowing for easy control and manipulation of LED strips. It uses a priority-based system
 * where multiple light requests can be active simultaneously, but only the highest priority
 * request is displayed on the lights.
 * 
 * <p>This class is implemented as a singleton and must be registered during robot initialization.
 * Once registered, it can be called from anywhere in the code (including outside of commands)
 * similar to a logger.
 * 
 * <p><b>Example Usage:</b>
 * <pre>{@code
 * // Register in RobotContainer
 * .add(LightStack.class, "LIGHTS", () -> {
 *     return new LightStack(CAN.CANDLE1, CAN.CANDLE2);
 * })
 * 
 * // Request light patterns from anywhere
 * LightStack.request(LightRequest.getCritical());
 * LightStack.request(LightRequest.getTeamRed());
 * LightStack.request(LightRequest.getDefault()
 *     .color(LightRequest.ORANGE)
 *     .animation(AnimationType.FADE, 1.0)
 *     .setSeconds(5.0));
 * LightStack.request(LightRequest.color(255, 0, 255)
 *     .animation(AnimationType.RAINBOW, 0.2)
 *     .setSeconds(10.0));
 * }</pre>
 * 
 * @author Team 2202
 */
public class LightStack extends SubsystemBase {

    /**
     * Enumeration of available LED animation types.
     * 
     * <p>Note: Some animations (RAINBOW, FIRE) ignore the color setting
     * and use their own built-in color patterns.
     */
    public enum AnimationType {
        /** Solid color with no animation */
        SOLID,
        /** Alternating on/off blinking pattern */
        BLINK,
        /** Smooth fade in and out */
        FADE,
        /** Rainbow color cycle animation */
        RAINBOW,
        /** Fire-like flickering animation */
        FIRE,
        /** Fast strobe effect */
        STROBE
    }

    /**
     * Represents a request to display a specific light pattern with priority and timing.
     * 
     * <p>Light requests use a fluent builder pattern, allowing for easy chaining of configuration
     * methods. Each request has a priority level, and when multiple requests are active,
     * only the highest priority request is displayed. If two requests have the same priority,
     * the most recently added one takes precedence.
     * 
     * <p>Requests can have a duration (in seconds) after which they automatically expire and
     * are removed from the stack. Use {@code Double.MAX_VALUE} for requests that should
     * remain active indefinitely.
     * 
     * <p><b>Example:</b>
     * <pre>{@code
     * LightRequest request = LightRequest.getDefault()
     *     .color(255, 0, 0)
     *     .animation(AnimationType.STROBE, 0.1)
     *     .setPriority(LightRequest.PRIORITY_HIGH)
     *     .setSeconds(5.0);
     * }</pre>
     */
    public static class LightRequest {
        /** Predefined color: Black (off) */
        public static final Color8Bit BLACK = new Color8Bit(0, 0, 0);
        /** Predefined color: White */
        public static final Color8Bit WHITE = new Color8Bit(255, 255, 255);
        /** Predefined color: Red */
        public static final Color8Bit RED = new Color8Bit(255, 0, 0);
        /** Predefined color: Green */
        public static final Color8Bit GREEN = new Color8Bit(0, 255, 0);
        /** Predefined color: Blue */
        public static final Color8Bit BLUE = new Color8Bit(0, 0, 255);
        /** Predefined color: Orange */
        public static final Color8Bit ORANGE = new Color8Bit(255, 145, 0);
        /** Predefined color: Yellow */
        public static final Color8Bit YELLOW = new Color8Bit(255, 255, 0);

        /** Priority level: Low (10) - typically for default/ambient lighting */
        public static final int PRIORITY_LOW = 10;
        /** Priority level: Normal (100) - typical game status indicators */
        public static final int PRIORITY_NORMAL = 100;
        /** Priority level: High (1000) - important game events */
        public static final int PRIORITY_HIGH = 1000;
        /** Priority level: Very High (10000) - critical game events */
        public static final int PRIORITY_VERYHIGH = 10000;
        /** Priority level: Critical (1000000) - emergency/safety alerts */
        public static final int PRIORITY_CRITICAL = 1000000;

        private int priority = PRIORITY_NORMAL;
        private Color8Bit color = WHITE;
        private int startLed = 0;
        private int length = 8;
        private int candleIndex = 0;
        private AnimationType animationType = AnimationType.SOLID;
        private double periodSeconds = 0.5;
        private double timeRemaining = Double.MAX_VALUE;

        /**
         * Private constructor to enforce use of factory methods.
         */
        private LightRequest() {
        }

        // --- Static Factory Methods ---
        
        /**
         * Creates a default light request with normal priority and white color.
         * 
         * @return a new LightRequest with default settings
         */
        public static LightRequest getDefault() {
            return new LightRequest();
        }

        /**
         * Creates a light request with white color.
         * 
         * @return a new LightRequest set to white
         */
        public static LightRequest getWhite() {
            return new LightRequest().color(WHITE);
        }

        /**
         * Creates a light request with red color.
         * 
         * @return a new LightRequest set to red
         */
        public static LightRequest getRed() {
            return new LightRequest().color(RED);
        }

        /**
         * Creates a light request with blue color.
         * 
         * @return a new LightRequest set to blue
         */
        public static LightRequest getBlue() {
            return new LightRequest().color(BLUE);
        }

        /**
         * Creates a low-priority red team color request that runs indefinitely.
         * 
         * <p>This is designed to be a fallback team color that displays when no other
         * higher-priority requests are active.
         * 
         * @return a new LightRequest for red alliance team color
         */
        public static LightRequest getTeamRed() {
            return new LightRequest().setPriority(PRIORITY_LOW).color(RED).setSeconds(Double.MAX_VALUE);
        }

        /**
         * Creates a low-priority blue team color request that runs indefinitely.
         * 
         * <p>This is designed to be a fallback team color that displays when no other
         * higher-priority requests are active.
         * 
         * @return a new LightRequest for blue alliance team color
         */
        public static LightRequest getTeamBlue() {
            return new LightRequest().setPriority(PRIORITY_LOW).color(BLUE).setSeconds(Double.MAX_VALUE);
        }

        /**
         * Creates a critical priority strobe alert in red.
         * 
         * <p>This is designed for emergency situations or critical alerts that should
         * override all other light patterns.
         * 
         * @return a new LightRequest with critical priority and red strobe animation
         */
        public static LightRequest getCritical() {
            return new LightRequest().setPriority(PRIORITY_CRITICAL).animation(AnimationType.STROBE, 0.1).color(RED);
        }

        // --- Fluent Modifiers ---
        
        /**
         * Sets the priority level for this light request.
         * 
         * <p>Higher priority requests will be displayed over lower priority ones.
         * Use the PRIORITY_* constants for standard priority levels.
         * 
         * @param priority the priority value (higher values = higher priority)
         * @return this LightRequest for method chaining
         */
        public LightRequest setPriority(int priority) {
            this.priority = priority;
            return this;
        }

        /**
         * Sets the color for this light request.
         * 
         * @param c the Color8Bit to display
         * @return this LightRequest for method chaining
         */
        public LightRequest color(Color8Bit c) {
            this.color = c;
            return this;
        }

        /**
         * Sets the color for this light request using RGB values.
         * 
         * @param r red component (0-255)
         * @param g green component (0-255)
         * @param b blue component (0-255)
         * @return this LightRequest for method chaining
         */
        public LightRequest color(int r, int g, int b) {
            return color(new Color8Bit(r, g, b));
        }

        /**
         * Sets the duration for this light request.
         * 
         * <p>After the specified duration, the request will expire and be removed
         * from the stack. Use {@code Double.MAX_VALUE} for requests that should
         * remain active indefinitely.
         * 
         * @param duration the duration in seconds
         * @return this LightRequest for method chaining
         */
        public LightRequest setSeconds(double duration) {
            this.timeRemaining = duration;
            return this;
        }

        /**
         * Sets the animation type with default speed of 0.5 seconds.
         * 
         * @param type the animation type to use
         * @return this LightRequest for method chaining
         */
        public LightRequest animation(AnimationType type) {
            return animation(type, 0.5);
        }

        /**
         * Sets the animation type and speed.
         * 
         * <p>The meaning of speedInSeconds varies by animation type:
         * <ul>
         *   <li>BLINK/STROBE: period between blinks</li>
         *   <li>FADE: duration of fade cycle</li>
         *   <li>RAINBOW/FIRE: animation speed scalar</li>
         * </ul>
         * 
         * @param type the animation type to use
         * @param speedInSeconds the animation speed/period in seconds
         * @return this LightRequest for method chaining
         */
        public LightRequest animation(AnimationType type, double speedInSeconds) {
            this.animationType = type;
            this.periodSeconds = speedInSeconds;
            return this;
        }

        /**
         * Specifies which CANdle device this request should control.
         * 
         * @param index the index of the CANdle device (0-based)
         * @return this LightRequest for method chaining
         */
        public LightRequest onDevice(int index) {
            this.candleIndex = index;
            return this;
        }

        /**
         * Specifies the range of LEDs to control on the device.
         * 
         * <p>This allows controlling a subset of LEDs on a longer LED strip.
         * 
         * @param start the starting LED index
         * @param numLeds the number of LEDs to control
         * @return this LightRequest for method chaining
         */
        public LightRequest range(int start, int numLeds) {
            this.startLed = start;
            this.length = numLeds;
            return this;
        }

        /**
         * Updates the remaining time for this request.
         * 
         * <p>This method is called internally by LightStack and should not be called directly.
         * 
         * @param deltaTime the time elapsed since last update in seconds
         */
        private void updateTime(double deltaTime) {
            if (this.timeRemaining != Double.MAX_VALUE) {
                this.timeRemaining -= deltaTime;
            }
        }

        /**
         * Checks if this request has expired.
         * 
         * <p>This method is called internally by LightStack and should not be called directly.
         * 
         * @return true if the request has expired, false otherwise
         */
        private boolean isFinished() {
            return this.timeRemaining <= 0;
        }

        /**
         * Compares this request with another to determine if they represent the same visual output.
         * 
         * <p>This is used to avoid redundantly sending the same pattern to the hardware.
         * Note: This does not compare timeRemaining or candleIndex.
         * 
         * @param other the LightRequest to compare with
         * @return true if the requests would produce the same visual output
         */
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

    /**
     * Internal class representing a single CANdle LED device.
     * 
     * <p>Each Device tracks the hardware interface, LED count, and the currently
     * displayed light request to optimize hardware communication.
     */
    private static class Device {
        /** The CANdle hardware interface */
        public final CANdle hardware;
        /** Number of LEDs on this device */
        public final int ledCount;
        /** The light request currently being displayed (null if none) */
        public LightRequest currentlyShowing = null;

        /**
         * Creates a Device with specified CAN ID and LED count.
         * 
         * @param id the CAN ID of the CANdle device
         * @param count the number of LEDs on this device
         */
        public Device(int id, int count) {
            this.hardware = new CANdle(id, "rio");
            this.ledCount = count;
        }

        /**
         * Creates a Device with specified CAN ID and default LED count of 8.
         * 
         * <p>8 LEDs is the default number of on-board LEDs on a standard CANdle device.
         * 
         * @param id the CAN ID of the CANdle device
         */
        public Device(int id) {
            this(id, 8);
        }
    }

    private final List<Device> candleList = new ArrayList<>();
    private final List<TreeMap<Integer, LightRequest>> stackList = new ArrayList<>();
    private double lastTime;
    private static LightStack instance;

    /**
     * Constructs a LightStack with multiple CANdle devices, each with specified LED counts.
     * 
     * <p>This constructor should only be called once during robot initialization.
     * Attempting to construct a second instance will throw an IllegalStateException.
     * 
     * @param candleIDandLength array of [CAN_ID, LED_COUNT] pairs for each CANdle device
     * @throws IllegalStateException if LightStack has already been constructed
     */
    public LightStack(int[][] candleIDandLength) {
        if (instance != null) {
            throw new IllegalStateException("LightStack constructor called twice!");
        }

        for (int[] config : candleIDandLength) {
            candleList.add(new Device(config[0], config[1]));
            stackList.add(new TreeMap<>());
        }
        this.lastTime = Timer.getFPGATimestamp();

        instance = this;
    }

    /**
     * Constructs a LightStack with multiple CANdle devices using default LED count of 8.
     * 
     * <p>This constructor should only be called once during robot initialization.
     * Attempting to construct a second instance will throw an IllegalStateException.
     * 
     * @param candleIds array of CAN IDs for each CANdle device
     * @throws IllegalStateException if LightStack has already been constructed
     */
    public LightStack(int[] candleIds) {
        if (instance != null) {
            throw new IllegalStateException("LightStack constructor called twice!");
        }

        for (int id : candleIds) {
            candleList.add(new Device(id));
            stackList.add(new TreeMap<>());
        }
        this.lastTime = Timer.getFPGATimestamp();

        instance = this;
    }

    /**
     * Gets the LightStack singleton instance.
     * 
     * @return the LightStack singleton
     * @throws IllegalStateException if called before the constructor has run
     */
    public static LightStack getInstance() {
        if (instance == null) {
            DataLogManager.log("CRITICAL: LightStack accessed before initialization!");
            throw new IllegalStateException(
                    "LightStack singleton accessed before it was initialized in RobotContainer. " +
                            "Check your construction order!");
        }
        return instance;
    }

    /**
     * Submits a light request to be displayed on the LED devices.
     * 
     * <p>The request is added to the priority stack for the specified device.
     * If multiple requests are active, the one with the highest priority will be displayed.
     * If the request's candleIndex is invalid, the request is silently ignored.
     * 
     * @param req the LightRequest to display
     */
    public static void request(LightRequest req) {
        if (req.candleIndex < instance.stackList.size()) {
            instance.stackList.get(req.candleIndex).put(req.priority, req);
        }
    }

    /**
     * Periodic update method called automatically by the command scheduler.
     * 
     * <p>This method:
     * <ul>
     *   <li>Updates timing for all active requests</li>
     *   <li>Removes expired requests from the stack</li>
     *   <li>Determines the highest priority request for each device</li>
     *   <li>Updates hardware only when the displayed pattern changes</li>
     *   <li>Shuts down LEDs during brownout conditions (below 7.0V)</li>
     * </ul>
     */
    @Override
    public void periodic() {
        double deltaTime = Timer.getFPGATimestamp() - lastTime;
        lastTime = Timer.getFPGATimestamp();

        for (int i = 0; i < candleList.size(); i++) {
            Device device = candleList.get(i);
            TreeMap<Integer, LightRequest> stack = stackList.get(i);

            // Brownout check - shut down lights to save power for critical systems
            double vbat = device.hardware.getSupplyVoltage().getValueAsDouble();
            if (vbat < 7.0) {
                device.hardware.setControl(new com.ctre.phoenix6.controls.NeutralOut());
                continue;
            }

            // Update timing and remove expired requests
            List<Integer> expiredPriorities = new ArrayList<>();
            for (Map.Entry<Integer, LightRequest> entry : stack.entrySet()) {
                entry.getValue().updateTime(deltaTime);
                if (entry.getValue().isFinished())
                    expiredPriorities.add(entry.getKey());
            }
            for (Integer p : expiredPriorities)
                stack.remove(p);

            // Get the highest priority request (or null if stack is empty)
            LightRequest winner = stack.isEmpty() ? null : stack.lastEntry().getValue();

            // Update hardware only if the pattern has changed
            if (winner == null) {
                if (device.currentlyShowing != null) {
                    device.hardware.setControl(new com.ctre.phoenix6.controls.NeutralOut());
                    device.currentlyShowing = null;
                }
            } else if (!winner.isSameAs(device.currentlyShowing)) {
                applyToCANdle(device.hardware, winner);
                device.currentlyShowing = winner;
            }
        }
    }

    /**
     * Applies a light request to the specified CANdle hardware device.
     * 
     * <p>This method translates the LightRequest into appropriate Phoenix 6 control objects
     * and sends them to the hardware. The device is first cleared with NeutralOut before
     * applying the new pattern.
     * 
     * @param candle the CANdle hardware device to control
     * @param req the LightRequest specifying the pattern to display
     */
    private void applyToCANdle(CANdle candle, LightRequest req) {
        // Clear the device first
        candle.setControl(new NeutralOut());

        // Prepare the color
        var targetColor = new RGBWColor(req.color.red, req.color.green, req.color.blue);

        // Apply the appropriate animation based on type
        switch (req.animationType) {
            case BLINK:
            case STROBE:
                candle.setControl(new StrobeAnimation(req.startLed, req.length)
                        .withColor(targetColor)
                        .withFrameRate(1.0 / Math.max(req.periodSeconds, 0.001)));
                break;

            case FADE:
                candle.setControl(new SingleFadeAnimation(req.startLed, req.length)
                        .withColor(targetColor)
                        .withFrameRate(1.0 / Math.max(req.periodSeconds, 0.001)));
                break;

            case RAINBOW:
                candle.setControl(new RainbowAnimation(req.startLed, req.length)
                        .withFrameRate(req.periodSeconds));
                break;

            case FIRE:
                candle.setControl(new FireAnimation(req.startLed, req.length)
                        .withFrameRate(req.periodSeconds));
                break;

            case SOLID:
            default:
                candle.setControl(new SolidColor(req.startLed, req.length).withColor(targetColor));
                break;
        }
    }
}