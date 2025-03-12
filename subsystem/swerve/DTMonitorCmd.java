// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.subsystem.swerve;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import static frc.lib2202.Constants.DEGperRAD;
/*
 * Watcher for SwerveDrivetrain and its vision data.
 *
 *  Only watches high level data, for module details see the tables for each of the modules.
 * 
 * @deprecated Use watcher inside SDT class
 */
@Deprecated
public class DTMonitorCmd extends WatcherCmd {
 
  // chassis velocity
  NetworkTableEntry radiansPerSecond;
  NetworkTableEntry xMetersPerSec;
  NetworkTableEntry yMetersPerSec;

  // accessors for drivetrain
  final DriveTrainInterface sdt;
  final ChassisConfig cc;


  public DTMonitorCmd() {
    sdt = RobotContainer.getSubsystem("drivetrain");
    cc = RobotContainer.getRobotSpecs().getChassisConfig();
    
    // use smartdashboard for complex objects
    var tname = getTableName();
    SmartDashboard.putData(tname + "/drive PIDF", cc.drivePIDF);
    SmartDashboard.putData(tname + "/angle PIDF", cc.anglePIDF);
  }

  @Override
  public String getTableName() {
    return SwerveDrivetrain.class.getSimpleName();
  }

  @Override
  public void ntcreate() {
    NetworkTable MonitorTable = getTable();
    radiansPerSecond = MonitorTable.getEntry("Vrot");
    xMetersPerSec = MonitorTable.getEntry("Vx ");
    yMetersPerSec = MonitorTable.getEntry("Vy ");    
  }

  @Override
  public void ntupdate() {
    // robot coordinates - speeds
    var speeds = sdt.getChassisSpeeds();
    radiansPerSecond.setDouble(speeds.omegaRadiansPerSecond * DEGperRAD);
    xMetersPerSec.setDouble(speeds.vxMetersPerSecond);
    yMetersPerSec.setDouble(speeds.vyMetersPerSecond);
  }
}
