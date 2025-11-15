package frc.lib2202.subsystem.UX;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.function.Supplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TrimTable - a sub-system to manage persistent values tied to network tables.
 * It that allow you to change the value on the network table and add to your value as 
 * an offset.
 * 
 * As a subsystem, the periodic() is automaticaly called. Previous version required you 
 * to hook the periodic somewhere in the robot loop.
 * 
 */
public class TrimTables extends SubsystemBase {
    final static String defaultTableName = "Default";  //used if a tableName isn't given
    final static int defaultCheckRate = 50; // only do callback check every N frames 50-> 1/sec

    // all our tables - multiple tables is better for organizing
    // being static, there can be only one instance of TrimTable SubSystem.
    final static ConcurrentMap<String, Table> trimTables = new ConcurrentHashMap<>();
    final int m_checkRate;
    int m_periodic_count = 0;

    public TrimTables() {
        this(defaultCheckRate);
    }

    public TrimTables(int checkRate) {
        this.m_checkRate = checkRate;        
    }

    @Override
    public void periodic() {
        // We expect trim table to be changed infrequently, so don't run callback checks
        // often
        if (++m_periodic_count % m_checkRate != 0)
            return;

        // monitor subs for new trim values if they have a callback
        for (Table tbl : trimTables.values()) {
            for (Trim t : tbl.m_trims) {
                double newval = t.m_trimSubscriber.get();
                if (newval != t.m_value) {
                    t.m_value = newval;
                    for (Supplier<Boolean> cb : t.m_callbacks) {
                        cb.get(); // don't care about returned bool
                    }
                }
            }
        }
    }

    // helper class to track different tables of Trims
    static class Table {
        final String tbl_root;
        final ArrayList<Trim> m_trims; // trims on this table
        final NetworkTable m_networkTable; // network table support

        private Table(String tableName) {
            tbl_root = "TrimTables/" + tableName ;
            this.m_trims = new ArrayList<Trim>();
            this.m_networkTable = NetworkTableInstance.getDefault().getTable(tbl_root);
            trimTables.putIfAbsent(tableName, this);
        }

        static Table get(String tableName) {
            synchronized (trimTables) {
                Table tbl = trimTables.get(tableName);
                if (tbl == null)
                    tbl = new Table(tableName);
                return tbl;
            }
        }
    } // class Table
    

    // Public API to add trims
    public static class Trim {
        // instance vars
        final Table m_table; // which table the trim is on
        final DoubleSubscriber m_trimSubscriber; // network table hook
        final ArrayList<Supplier<Boolean>> m_callbacks; // functions to call on change
        double m_value;

        // default table - keep older api
        public Trim(String name) {
            this(defaultTableName, name, null, 0.0);
        }

        public Trim(String name, double default_trim) {
            this(defaultTableName, name, null, default_trim);
        }

        public Trim(String name, Supplier<Boolean> callback, double default_trim) {
            this(defaultTableName, name, callback, default_trim);
        }

        // Support Trims on their own table instead of default
        public Trim(String tableName, String name) {
            this(tableName, name, null, 0.0);
        }

        public Trim(String tableName, String name, double default_trim) {
            this(tableName, name, null, default_trim);
        }

        /**
         * Trim constructor
         * 
         * Usage:
         *      import frc.lib2202.subsystem.UX.TrimTable.Trim;
         * 
         *      Trim myTrim = new Trim("MySSTrims", "MyTrim", null, 5.0);
         * 
         * @param trimTableName - tableName to organize under
         * @param trimName      - name 
         * @param callback      - optional, can have multiple callbacks
         * @param default_trim  - inital value to put in to table if entry isn't there in persistence file
         */
        
         public Trim(String trimTableName, String trimName, Supplier<Boolean> callback, double default_trim) {
            this.m_callbacks = new ArrayList<Supplier<Boolean>>();
            this.m_table = Table.get(trimTableName); // constructs table as needed, static fuction
            this.m_table.m_trims.add(this);
            // create the topic, and subscribe
            DoubleTopic d_topic = m_table.m_networkTable.getDoubleTopic(trimName);
            m_trimSubscriber = d_topic.subscribe(default_trim);
            // get the persisted value, if there is one
            m_value = m_trimSubscriber.get();
            // publish & persist
            d_topic.publish().set(this.m_value);
            d_topic.setPersistent(true);

            addChangeCallback(callback); 
            
        }

        public double getValue() {
            m_value = m_trimSubscriber.get();
            return m_value;
        }

        public double getValue(double value) {
            m_value = m_trimSubscriber.get();
            return value + m_value;
        }

        /**
         * Adds a callback function for the trim. Function will be called if the value
         * changes.
         * Make sure Trim.periodic() is called somewhere in the robot loop.
         * You can have multiple change callbacks by calling this function multiple time
         * with different
         * functions.
         * 
         * @param callback - Supplier<Boolean> is use, returned bool has no meaning,
         *                 just an easy func to define.
         * @return
         */
        public Trim addChangeCallback(Supplier<Boolean> callback) {
            if (callback != null)
                m_callbacks.add(callback);
            return this;
        }
    }

}
