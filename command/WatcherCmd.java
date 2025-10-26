// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.command;

import static frc.lib2202.Constants.DEGperRAD;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.util.NetworkTableUtil;

public abstract class WatcherCmd extends Command implements NetworkTableUtil{
  /** Creates a new Watcher. */
  final NetworkTable table;
  final String name;
   
  public WatcherCmd() {
    this.name = getTableName();
    this.table = NetworkTableInstance.getDefault().getTable(name);
    ntcreate();
    this.runsWhenDisabled();
    this.schedule();
  }

  public NetworkTable getTable() {
    return table;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ntupdate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }

  //formating functions to round to n digits
  public static double fmt0(double x) {       
    return Math.round(x);
  }
  final static double scale1 = 10.0; 
  public static double fmt1(double x) {       
    return Math.round(x*scale1)/scale1;
  }

  final static double scale2 = 100.0; 
  public static double fmt2(double x) {       
    return Math.round(x*scale2)/scale2;
  }

  public static double fmt2toDeg(double x) {   
    final double deg_scale2 = 100.0*DEGperRAD;
    return Math.round(x*deg_scale2)/scale2;
  }

}
