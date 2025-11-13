// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.command;

import static frc.lib2202.Constants.DEGperRAD;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Notes:
 * removed "implements NetworkTableUtil" and moved interface inside.
 * removed unused/unneeded code to simplify
 * better default tablename using subsystem getName() if found.
 */
public abstract class WatcherCmd extends Command {
  /** Creates a new Watcher. */
  final NetworkTable table;
  //final String name;

  public WatcherCmd() {
    //this.name = getTableName();
    this.table = NetworkTableInstance.getDefault().getTable(getTableName());
    ntcreate();
    // dont need to call this - this.runsWhenDisabled();
    this.schedule();
  }

  /**
   * Returns whether the network table entries for this class are necessary for
   * comp
   * 
   * @return whether the NTEs for this class are necessary for comps
   */
  public boolean necessaryForCompetition() {
    return false;
  }

  /**
   * Create NetworkTables here - your watcher must implement
   */
  abstract public void ntcreate();

  /**
   * Update NetworkTables here - your watcher must implement
   */
  abstract public void ntupdate();

  /*
   * Tell us who you are for the table
   */
  public String getTableName() {
    // use reflection to get the subsystem this watcher belongs to
    // and use its getName() to use as the table. This should cover most cases
    // where the sub-system is given a name.
    // if the reflection fails, fall back to naming based on watchername.
    // if none of this works, override this method with whatever you want.
    try {
      Class<?> decl_clz = this.getClass().getDeclaringClass();
      // this$0 is the outer field created when the watcher is constructed
      Field outerInstanceField = this.getClass().getDeclaredField("this$0");
      outerInstanceField.setAccessible(true); 
      //get the actual object
      Object outer_obj = outerInstanceField.get(this);
      //get the getName() method so we can call it
      Method m = decl_clz.getMethod("getName", (Class<?>[])null);
      var outer_inst_name = m.invoke(outer_obj);
      return outer_inst_name.toString();

    } catch (NoSuchMethodException | NoSuchFieldException |
            InvocationTargetException | IllegalAccessException |
            IllegalArgumentException e) { 
    }
    // reflect failed, move on 
    return this.getClass().getSimpleName();
  };

  protected NetworkTable getTable() {
    return table;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ntupdate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // formating functions to round to n digits
  public static double fmt0(double x) {
    return Math.round(x);
  }

  final static double scale1 = 10.0;

  public static double fmt1(double x) {
    return Math.round(x * scale1) / scale1;
  }

  final static double scale2 = 100.0;

  public static double fmt2(double x) {
    return Math.round(x * scale2) / scale2;
  }

  public static double fmt2toDeg(double x) {
    final double deg_scale2 = 100.0 * DEGperRAD;
    return Math.round(x * deg_scale2) / scale2;
  }

}
