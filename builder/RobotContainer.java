// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.builder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This class handles the initialization of the selected robot. Selection is based on
 * the serial number read off the RoboRio.  Set up all the potential RobotSpec_<botName>
 * files in Main.java.  Each RobotSpec contains all the data for that robot and
 * the list of sub-systems and commands that will be constructed AFTER the 
 * needed spec is identified by the RIO serial number.
 */
public class RobotContainer {
  public String LIB2202_VERSION = "1.0.4";   // version

  static RobotContainer rc;   //singleton

  //selected config (list of sub-systems) identified by RIO serial number
  final SubsystemConfig m_subsystemConfig;
  final SendableChooser<Command> m_autoChooser;
  
  // Static API for accessing the robot's sub-systems.

  // The following methods are unchecked, but the SystemConfig class does
  // check the types internally.

  /** Use the string name when there are multiple instances of a subsystem,
   * for example, having multiple limelights or odometry estimators.
  */
  @SuppressWarnings("unchecked")
  public static <T> T getSubsystem(String name) {
    return (T) rc.m_subsystemConfig.getSubsystem(name);
  }

  @SuppressWarnings("unchecked")
  public static <T extends Subsystem> T getSubsystemOrNull(String name) {
    return (T) rc.m_subsystemConfig.getObjectOrNull(name);
  }

  // Use this when there is only one instance of the Subsystem - preferred
  @SuppressWarnings("unchecked")
  public static <T extends Subsystem> T getSubsystem(Class<T> clz) {
    return (T) rc.m_subsystemConfig.getSubsystem(clz);
  }

  // Use this when there is only one instance of the Subsystem and can deal with
  // nulls in the context. It bypasses NPE checks, so you can handle configurations
  // that may not have the requested subsystem.
  @SuppressWarnings("unchecked")
  public static <T extends Subsystem> T getSubsystemOrNull(Class<T> clz) {
    return (T) rc.m_subsystemConfig.getObjectOrNull(clz.getSimpleName());
  }

  // Use this form when the RobotContainer object is NOT a Subsystem but some
  // other type of object.
  @SuppressWarnings("unchecked")
  public static <T> T getObject(String name) {
    return (T) rc.m_subsystemConfig.getObject(name);
  }

  // Use this form when the RobotContainer object is NOT a Subsystem, and you can
  // deal with nulls.
  @SuppressWarnings("unchecked")
  public static <T> T getObjectOrNull(String name) {
    return (T) rc.m_subsystemConfig.getObjectOrNull(name);
  }

  public static boolean hasSubsystem(Class<? extends Subsystem> clz) {
    return rc.m_subsystemConfig.hasSubsystem(clz);
  }

  public static IRobotSpec getRobotSpecs() {
    return rc.m_subsystemConfig.getRobotSpec();
  }

  public static String getRobotName() {
    return rc.m_subsystemConfig.m_robot_name;
  }

  /**
   * The container for the robot.
   * 
   * You likely shouldn't need to edit this file.  
   */
  public RobotContainer() {
    System.out.println("***Running lib2202 version "+ LIB2202_VERSION + " ***");
    RobotContainer.rc = this;
    // use serial number to set the proper config, use env or static set in Main.java
    // For sim debug, set in Debug:main powershell:   $env:serialnum ='123412341234'   
    String serialnum =  System.getenv("serialnum");
    m_subsystemConfig = SubsystemConfig.SetConfig(serialnum);
    IRobotSpec spec = getRobotSpecs();
    SubsystemConfig.constructAll();
    // Quiet some of the noise
    DriverStation.silenceJoystickConnectionWarning(true);
    
    // continue with rest of the robot initialization
    spec.setBindings();
    spec.setupRegisteredCommands();
    m_autoChooser = spec.getChooser();
    spec.setDefaultCommands();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return (m_autoChooser != null) ? m_autoChooser.getSelected() :  null;
  }

}