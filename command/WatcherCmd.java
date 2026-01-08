// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.command;

import static frc.lib2202.Constants.DEGperRAD;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Notes:
 * removed "implements NetworkTableUtil" and moved interface inside this class.
 * removed unused/unneeded code to simplify
 * better default tablename using subsystem getName() if found.
 * added addEntry form to simplify table stuff
 * WIP - convert to nt4 pub/sub api
 */
public abstract class WatcherCmd extends Command {
  /** Creates a new Watcher. */
  final NetworkTable table;
  final List< Entry> entries = new ArrayList<Entry>();
  
  public WatcherCmd() {
    String name = getTableName();
    //better name for the watcher cmd
    setName(name+":WatchCmd");
    this.table = NetworkTableInstance.getDefault().getTable(name);
    //auto schedule the watcher, if we create it, run it.
    CommandScheduler.getInstance().schedule(this);
  }

  // addEntry functions
  public void addEntry(String name, Supplier<?> func, int N) {
    entries.add(new Entry(name, func, N));   
  }
  //keep all the digits
  public void addEntry(String name, Supplier<?> func) {
    addEntry(name, func, -1);
  }

  /**
   * Returns whether the network table entries for this class are necessary for
   * competition - wip
   * 
   * @return whether the NTEs for this class are necessary for comps
   */
  public boolean necessaryForCompetition() {
    return false;
  }

  /**
   * Create NetworkTables here via override (old pattern)
   * or use addEntry() and it will be handled and this no-op funcion is used.
   */
  public void ntcreate() { }

  /**
   * Update NetworkTables, if you need something other than
   * the basics you need to override ntcreate() and ntupdate().
   * you may call super().ntcreate()/ntupdate() in your custom 
   * implementation to support the entries created via addEntry().
   */
  public void ntupdate(){
    for ( Entry entry : entries) {
      Object value = entry.supplier.get();

      if (entry.N < 0) {
        // publish what we have, no rounding or is some non-formatted type        
        entry.publisher.setValue(value);
        continue;
      }
      // deal with rounded types, basic double or float
      if (value instanceof Double) {
        double dvalue = fmt(entry.N, (Double)value);
        entry.publisher.setDouble(dvalue);        
      } else if (value instanceof Float) {       
        float fvalue_rd = (float) fmt(entry.N, (Float)value);
        entry.publisher.setFloat(fvalue_rd);      
      }
    }
  }

  /*
   * GetTableName()
   * 
   * Uses reflection to get the parent subsystem (or other clas) this watcher belongs 
   * to and use its subsystem::getName() for the table's name. This should cover most
   * cases.
   * 
   * Reflection was used to avoid chaning the constructor api which would need
   * a reference to the parent object.
   * 
   * if the reflection fails, fall back to naming based on watchername.
   * if none of this works, override this method with whatever you want.
   */
  public String getTableName() {
    Class<?> decl_clz = this.getClass().getDeclaringClass();
    try {
      // this is the outer field created when the watcher is constructed
      // see if we can use it Subsystem::getName for the table name.
      Field outerInstanceField = this.getClass().getDeclaredField("this$0");
      outerInstanceField.setAccessible(true); 
      //get the actual object, to call getName on.
      Object outer_obj = outerInstanceField.get(this);
      //get the getName() method so we can call it
      Method m = decl_clz.getMethod("getName", (Class<?>[])null);
      var outer_inst_name = m.invoke(outer_obj);
      return outer_inst_name.toString();

    } catch (NoSuchMethodException | NoSuchFieldException |
            InvocationTargetException | IllegalAccessException | IllegalArgumentException e) 
            { /* do nothing */ }
    // reflect failed, likely not a sub-system 
    // move on with declaring class's simple name or this class' simplename if decl_clz is null
    return (decl_clz != null) ? decl_clz.getSimpleName() : this.getClass().getSimpleName();
  };

  protected NetworkTable getTable() {
    return table;
  }

  @Override
  public void initialize(){
    //defer table create to initialize so all entries have been added after ctor finishes
    ntcreate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ntupdate();
  }

  @Override
  public boolean isFinished() {
    return false;   // watchers never end
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;   // watchers can always update their NT values.
  }


  final static double[] scales = {1.0, 10.0, 100.0, 1000.0, 10000.0};
  
  // formating functions to round to n digits
  public static double fmt(int N, double x) {
    double scale = scales[N];
    return Math.round(x * scale) / scale;
  }

  // formating functions to round to n digits
  public static double fmt0(double x) {
    return Math.round(x);
  }
  
  public static double fmt1(double x) {
    return fmt(1, x);
  }

  public static double fmt2(double x) {
    return  fmt(2, x);
  }

  public static double fmt2toDeg(double x) {
    final double deg_scale2 = 100.0 * DEGperRAD;
    return Math.round(x * deg_scale2) / 100.0;
  }


  // WIP - easier entry constructor api using Supplier<>
  class Entry {
    //NT4
    final GenericPublisher publisher;
    final Supplier<?> supplier;  //function to give supply data value
    final int N;                 //digits to round to

    Entry(String entryName, Supplier<?> func, int N){
      this.supplier = func;
      this.N = (N < scales.length - 1) ? N : -1;   // prevent out of bounds
      
      //NT4
      Object obj = supplier.get();
      String nt_type = NetworkTableType.getStringFromObject(obj); 
      Topic topic = table.getTopic(entryName);
      this.publisher = topic.genericPublish(nt_type);

      //issue warning about unknown supplier types
      if (nt_type.equals("")) {
        String type_clz_name = obj.getClass().getSimpleName();
        System.out.format("Warning WatcherCmd for %s doesn't support Supplier<%s>,%n"+
                        "Use custom ntcreate()/ntupdate() to support.%n",
                        getTableName(), type_clz_name);
      }
    }

    Entry(String entryName, Supplier<?> func){      
      this(entryName, func, -1);  //default to un-rounded value
    }

  }

}
