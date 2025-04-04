// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.builder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public String LIB2202_VERSION = "1.0.3";

  // The robot's subsystems and commands are defined here...
  static RobotContainer rc;   //singleton

  final SubsystemConfig subsystemConfig;
  final SendableChooser<Command> autoChooser;
  
  // support old accessor for Robot's container 
  @Deprecated
  public static RobotContainer RC() {
    return rc;
  }

  // The following methods are unchecked, but the SystemConfig class does
  // check the types.
  // Use the string name when there are multiple instance of the subsystem
  @SuppressWarnings("unchecked")
  public static <T> T getSubsystem(String name) {
    return (T) rc.subsystemConfig.getSubsystem(name);
  }

  @SuppressWarnings("unchecked")
  public static <T extends Subsystem> T getSubsystemOrNull(String name) {
    return (T) rc.subsystemConfig.getObjectOrNull(name);
  }

  // Use this when there is only one instance of the Subsystem - preferred
  @SuppressWarnings("unchecked")
  public static <T extends Subsystem> T getSubsystem(Class<T> clz) {
    return (T) rc.subsystemConfig.getSubsystem(clz);
  }

  // Use this when there is only one instance of the Subsystem and can deal with
  // nulls
  // in the context. It bypasses NPE checks. Know what you are doing.
  @SuppressWarnings("unchecked")
  public static <T extends Subsystem> T getSubsystemOrNull(Class<T> clz) {
    return (T) rc.subsystemConfig.getObjectOrNull(clz.getSimpleName());
  }

  // Use this form when the RobotContainer object is NOT a Subsystem
  @SuppressWarnings("unchecked")
  public static <T> T getObject(String name) {
    return (T) rc.subsystemConfig.getObject(name);
  }

  // Use this form when the RobotContainer object is NOT a Subsystem, and you can
  // deal with nulls
  @SuppressWarnings("unchecked")
  public static <T> T getObjectOrNull(String name) {
    return (T) rc.subsystemConfig.getObjectOrNull(name);
  }

  public static boolean hasSubsystem(Class<? extends Subsystem> clz) {
    return rc.subsystemConfig.hasSubsystem(clz);
  }

  public static IRobotSpec getRobotSpecs() {
    return rc.subsystemConfig.getRobotSpec();
  }

  public static String getRobotName() {
    return rc.subsystemConfig.name;
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
    String serialnum =  System.getenv("serialnum");
    //For sim debug, set in Debug:main powershell:   $env:serialnum ='123412341234'   

    subsystemConfig = SubsystemConfig.SetConfig(serialnum);
    SubsystemConfig.constructAll();

    // Quiet some of the noise
    DriverStation.silenceJoystickConnectionWarning(true);

    getRobotSpecs().setBindings();
    getRobotSpecs().setupRegisteredCommands();
    autoChooser = getRobotSpecs().getChooser();
    getRobotSpecs().setDefaultCommands();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return (autoChooser != null) ? autoChooser.getSelected() :  null;
  }

}