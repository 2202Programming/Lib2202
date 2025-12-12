package frc.lib2202.builder;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;

/*
* SubsystemConfig - handle different flavors of robot  sub-system building.
* Objects will be constructed in order they are added. This is critical for
* systems like Drivetrain which requires Sensors and maybe Vision to be 
* available.
*
*  See Configs.java for correct usage for different platforms we have.
*/
public class SubsystemConfig {
    static ShuffleboardTab SSTab;

    // track all the possible SubsystemConfigs created for later lookup by serial
    // number
    static List<SubsystemConfig> allConfigs = new ArrayList<SubsystemConfig>();
    static SubsystemConfig selectedConfig = null;

    // Member vars
    final String m_serialNumber;
    final String m_robot_name;
    IRobotSpec m_robotSpec = null;
    LinkedHashMap<String, SubsystemDefinition<?>> m_robot_parts = new LinkedHashMap<>();

    public SubsystemConfig(String robot_name, String serialNumber) {
        this.m_robot_name = robot_name;
        this.m_serialNumber = serialNumber;
        allConfigs.add(this);
    }

    static SubsystemConfig getSelectedConfig() {
        if (selectedConfig == null) {
            throw new RuntimeException("SetConfig(<serial number>) has not been set.");
        }
        return selectedConfig;
    }

    /**
     * SetConfg - sets the serial number and looks up the robot configuration to
     * use.
     * 
     * @param serialNumber normally read from RoboRIO or env:serialnum when
     *                     debugging
     * @return SubsystemConfig found by serialNumber
     */
    static SubsystemConfig SetConfig(String serialNumber) {
        System.out.println("***RoboRio SERIAL NUM: '" + serialNumber + "' ***");

        if (selectedConfig != null) {
            throw new RuntimeException(
                "Multiple SetConfig() calls, called again with '" + serialNumber +"'.\n" +
                "SetConfig() was previously called with " + selectedConfig.m_serialNumber + " for '" +
                selectedConfig.m_robot_name + "'.\n" +
                "Check your code for multiple SetConfig() calls in RobotSpec_<>.java file.");
        }

        for (SubsystemConfig ssConfig : allConfigs) {
            if (serialNumber.equals(ssConfig.m_serialNumber)) {
                selectedConfig = ssConfig;
                break;
            }
        }

        // a config should be selected if not complain and crash
        if (selectedConfig == null) {
            throw new RuntimeException("No SubsystemConfig matching serial number '" + serialNumber + "' found." +
                    "\nCheck your RobotSpec_<>.java and Main.java for RobotSpec_<> construction.");
        }

        System.out.println("***** Robot identified as: '" + selectedConfig.m_robot_name +"' *****");
        return selectedConfig;
    }

    /* SubsystemDefinition keeps class and subsystem for use in a name map */
    static class SubsystemDefinition<T extends Object> {
        Class<T> m_Class;
        Object m_obj = null;
        Supplier<Object> m_factory = null;
        String m_alias = null;

        // container for Subsystem Class and instance object
        SubsystemDefinition(Class<T> clz) {
            m_Class = clz;
        }

        SubsystemDefinition(Class<T> clz, String alias) {
            m_Class = clz;
            m_alias = alias;
        }

        @SuppressWarnings("unchecked")
        SubsystemDefinition(T obj) {
            m_Class = (Class<T>) obj.getClass();
            m_obj = obj; // already constructed
        }

        SubsystemDefinition(Class<T> clz, Supplier<Object> factory) {
            m_Class = clz;
            m_factory = factory;
        }

        SubsystemDefinition(Class<T> clz, String alias, Supplier<Object> factory) {
            m_Class = clz;
            m_factory = factory;
            m_alias = alias;
        }

        // Construct the subsystem using the default constructor or supplied factory method.
        void construct() {
            // already created, move on
            if (m_obj != null) return;

            // use a no-args constructor for anything without a factory lambda
            try {
                // Use the no-args constructor or factory to create instance
                m_obj = (m_factory != null) ? 
                    m_factory.get() : 
                    m_Class.getDeclaredConstructor().newInstance();
                
                // add sendables to SSTab on Elastic/SmartDashboard
                if (m_obj instanceof Subsystem) {
                   // SmartDashboard.putData((Sendable)m_obj);
                    SSTab.add((Sendable)m_obj);                   
                }

            } catch (NoSuchMethodException e) {
                System.out.println("*** Problem creating " + m_Class.getSimpleName()
                        + " a no-arg constructor is required.  Object not created. ***");
                e.printStackTrace();

            } catch (InstantiationException | IllegalAccessException | IllegalArgumentException | 
                    SecurityException | InvocationTargetException e) {
                // handle other cases that shouldn't happen for Subsystems
                System.out.println("***Problem creating Object " +
                        m_Class.getName() + "... Subsystem not created.***");                
                e.printStackTrace();
                System.out.println("Continuing with remaining objects.");
            }
        }
    }

    /*
     * An alias gives an existing Object another name for lookup. Either clz object
     * or name may be used to lookup objects.
     * 
     * This is useful for cases where a command can work with different types of
     * Subsystems.
     * 
     * For example, different drivetrains can support pathing or rotation resets,
     * but should not be tied to a specif class. DRIVETRAINs fit this use case.
     * An interface is the preferred way for a command to use the lookup result.
     * 
     * add() addAlias() are public so they can be used in RobotSpec_<> files.
     * get() are protected, the public API is exposed by RobotContainer's get()
     * methods.
     * 
     */

    /**
     * addAlias()
     * Creates an alias name for the given subsystem. Both class and name are
     * locatable.
     * Use when you need to lookup by class name or alias because
     * some commands prefer alias because the command uses a compatible interface.
     * No args constructor is used.
     * 
     * @param <T>
     * @param clz
     * @param alias
     * @return
     */
    public <T> SubsystemConfig addAlias(Class<T> clz, String alias) {
        String name = clz.getSimpleName();
        var ssd = new SubsystemDefinition<T>(clz);
        ssd.m_alias = alias;
        put(name, ssd);
        put(alias, ssd); // ailas also points to this ssd, so either name finds SS
        return this;
    }

    /**
     * addAlias()
     * creates an alias name for the given subsystem.
     * Use when you need to lookup by class name or alias because
     * some commands prefer alias because the command uses a compatible interface.
     * Construction is done in the supplied factory function.
     * Use when you need to specify args or do other post construction setup.
     * 
     * @param <T>     Type
     * @param clz     Class object, i.e. className.class
     * @param alias   Lookup alias
     * @param factory method to construct instance, returns T object.
     * @return this SubsystemConfig for call chaining
     */
    public <T> SubsystemConfig addAlias(Class<T> clz, String alias, Supplier<Object> factory) {
        String name = clz.getSimpleName();
        var ssd = new SubsystemDefinition<T>(clz, alias, factory);
        put(name, ssd);
        put(alias, ssd); // ailas also points to this ssd, so either clz or alias finds SS
        return this;
    }

    /**
     * add()
     * Simple form of adding a subsystem to the configuration.
     * Lookup by class object.
     * The no-args constructor is used
     * 
     * @param <T> Type
     * @param clz Class object, i.e. className.class
     * @return this SubsystemConfig for call chaining
     */
    public <T> SubsystemConfig add(Class<T> clz) {
        String name = clz.getSimpleName();
        var ssd = new SubsystemDefinition<T>(clz);
        put(name, ssd);
        return this;
    }

    /**
     * add()
     * Another simple form of adding a subsystem to the configuration but
     * with a lookup name.
     * You can't lookup by class object, lookup by name only.
     * Use this form when there are multiple instances of the same subsystem (rare)
     * and you need to identify them uniquely.
     * Each name must be unique.
     * The no-args constructor is used.
     * 
     * @param <T>  Type
     * @param clz  Class object, i.e. className.class
     * @param name lookup name, must be unique
     * @return this SubsystemConfig for call chaining
     */
    public <T> SubsystemConfig add(Class<T> clz, String name) {
        put(name, new SubsystemDefinition<T>(clz, name));
        return this;
    }

    /**
     * add()
     * Add an already constructed object. (rarely needed)
     * 
     * @param <T>      Type of the object being added.
     * @param instance pre-constructed object, ctor not called
     * @param name     lookup name of the instance
     * @return
     */
    public <T> SubsystemConfig add(T instance, String name) {
        put(name, new SubsystemDefinition<T>(instance));
        return this;
    }

    /**
     * add()
     * Facory adds must have a Type and a name
     * 
     * @param <T>     type
     * @param clz     class object
     * @param name    lookup name
     * @param factory method to call for constructing instance
     * @return
     */
    public <T> SubsystemConfig add(Class<T> clz, String name, Supplier<Object> factory) {
        put(name, new SubsystemDefinition<T>(clz, name, factory));
        return this;
    }

    /**
     * add()
     * Facory adds must have a type, use class name since none given
     * @param <T>
     * @param clz
     * @param factory
     * @return
     */
    public <T> SubsystemConfig add(Class<T> clz, Supplier<Object> factory) {
        String name = clz.getSimpleName();
        put(name, new SubsystemDefinition<T>(clz, factory));
        return this;
    }

    // Getters used in commands or Subsystems to lookup needed robot objects
    // These are not public, they are exposed in RobotContainer static get() 
    // methods.

    /**
     * Returns the subsystem found by the given name.
     * Only subsystems will be returned. Returns null if object is found but not
     * of type Subsystem.
     * 
     * @param name lookup name
     * @return Subsystem instance or null if found but not a SubSystem. 
     *         NPE on not found.
     */
    Subsystem getSubsystem(String name) {
        var ssd = get(name);
        return (ssd.m_obj instanceof Subsystem) ? (Subsystem) ssd.m_obj : null;
    }

    /**
     * Returns a subsystem found by the class object.
     * 
     * @param <T> object's type
     * @param clz class object to lookup
     * @return SubSystem or NPE thrown
     */
    @SuppressWarnings("unchecked")
    <T extends Subsystem> T getSubsystem(Class<? extends Subsystem> clz) {
        String name = clz.getSimpleName();
        var ssd = get(name);
        return (ssd.m_obj instanceof Subsystem) ? (T) ssd.m_obj : null;
    }

    /**
     * Any type of robot object will be returned, including Subsystems.
     * Lookup name must be unique to the configuration.
     *
     * @param <T>  object's type
     * @param name lookup name
     * @return object or throws NPE on failure
     */
    @SuppressWarnings("unchecked")
    <T> T getObject(String name) {
        var ssd = get(name);
        return (T) ssd.m_obj;
    }

    /**
     * Any type of robot object will be returned, including Subsystems.
     * Lookup name must be unique to the configuration.
     * Null returned if not found
     * 
     * @param <T>  object's type
     * @param name lookup name
     * @return object or null
     */
    @SuppressWarnings("unchecked")
    <T> T getObjectOrNull(String name) {
        var ssd = m_robot_parts.get(name);
        return (ssd != null) ? (T) ssd.m_obj : null;
    }

    // Only subsystems will be returned
    @SuppressWarnings("unchecked")
    <T extends Subsystem> T getSubsystemOrNull(Class<? extends Subsystem> clz) {
        String name = clz.getSimpleName();
        var ssd = m_robot_parts.get(name);
        return (ssd.m_obj instanceof Subsystem) ? (T) ssd.m_obj : null;
    }

    boolean has(String name) {
        return m_robot_parts.containsKey(name);
    }

    // for when it's not a Subsystem, and a factory wasn't used
    boolean has(Class<?> clz) {
        String n = clz.getSimpleName();
        return has(n);
    }

    // for when it must be a subsystem
    boolean hasSubsystem(Class<?> clz) {
        if (!clz.isNestmateOf(Subsystem.class))
            return false; // clz not Subsystem
        String n = clz.getSimpleName();
        return has(n);
    }

    public SubsystemConfig setRobotSpec(IRobotSpec spec) {
        m_robotSpec = spec;
        return this;
    }
    
    IRobotSpec getRobotSpec() {
        if (m_robotSpec == null) {
            throw new RuntimeException("RobotSpec not setup for robot: " + this.m_robot_name + "\n" +
                    "Check your RobotSpec_" + this.m_robot_name + ".java file, and " +
                    "add 'ssConfig.setRobotSpec(this)' in the default constructor.");
        }
        return m_robotSpec;
    }

    /*
     * constructAll() - calls all the subsystem constructors or factory methods
     * if they haven't been initialized by adding a pre-constructed object.
     * 
     * This is called in RobotContainer for its only its system config after
     * all the robot specs are setup and the serialNumber is found.
     */
    static void constructAll() {
        SSTab = Shuffleboard.getTab("SubSystem Info");
        var cfg = getSelectedConfig();

        // wait for Phoenix library initialization on other threads to complete
        System.out.println("Waiting 5 seconds for other threads to initialize, then constructing: " +
                cfg.m_robot_name + ":" + cfg.m_serialNumber);
        sleep(5000);
        System.out.println("Constructing " + selectedConfig.m_robot_name);
        for (Map.Entry<String, SubsystemDefinition<?>> entry : cfg.m_robot_parts.entrySet()) {
            System.out.println("    Constructing " + entry.getKey() + 
                " as instance of " + entry.getValue().m_Class.getSimpleName());
            entry.getValue().construct();
        }
    }

    private void put(String name, SubsystemDefinition<?> ssd) {
        // see if name exist, if so that is a problem as names must be unique
        if (m_robot_parts.containsKey(name)) {
            System.out.println("*********************************************\n"
                    + "SubsystemConfig warning: Configs.java contains DUPLICATE NAME "
                    + name + "of Class " + ssd.m_Class.getCanonicalName()
                    + " duplicate will not be created."
                    + "*********************************************\n");
        }
        m_robot_parts.put(name, ssd);
    }

    private SubsystemDefinition<?> get(String name) {
        var ssd = m_robot_parts.get(name);
        if (ssd != null)
            return ssd;

        // look back on stack to improve error message
        StackTraceElement lastMethod = Thread.currentThread().getStackTrace()[4];
        String fileName = lastMethod.getFileName();
        String className = lastMethod.getClassName();
        String methodName = lastMethod.getMethodName();
        int line = lastMethod.getLineNumber();

        // name doesn't exist, fail hard and fast
        System.out.println("SubsystemConfig Error: '" + name + "' does not exist.\n"
                + "\t" + fileName + " #"+ line +" in " + className + "." + methodName + "()\n"
                + "\tCheck your RobotSpec_<>.java file\n"
                + "\tThrowing NPE because needed object isn't available.\n"
                + "\tUse getSubsystemOrNull() if null is acceptable.\n");
        throw new NullPointerException();
    }

    static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception e) {
        }
    }

}
