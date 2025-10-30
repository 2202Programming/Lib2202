// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.builder;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;

/**
 * InnerIRobotSpec
 */
public interface IRobotSpec {

    // basic robot speeds
    default public RobotLimits getRobotLimits() {
        RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(360.0));
        return robotLimits;
    }

    // Sensors needed for drivetrain
    default public IHeadingProvider getHeadingProvider() {
        return null;
    }

    // swerve specs
    default public ChassisConfig getChassisConfig() {
        return null;
    }

    default public ModuleConfig[] getModuleConfigs() {
        return null;
    }

    // deploy directory - this is only needed when supporting multiple robot and
    // there data is in a sub-folder of deploy, typically the year "2025" and imported
    // into a multi-year repo. 
    // If overidden, contents like pathplaner are copied up one level at during init
    // so pathing folder and other data are in the expected location for that robot.
    // See TODO...
    default public String getDeployDirectory() {
        return null;
    }

    // bindings
    default public void setBindings() {
    }

    // default public boolean burnFlash(){ return true;};

    // Setup registered commands
    default public void setupRegisteredCommands() {
    };

    // Setup registered commands
    default public SendableChooser<Command> getChooser() {
        return null;
    }

    default public void setDefaultCommands() {
    }

    // Most of the time these are not needed, but you can override them in your robot's 
    // spec file on rare cases you want to have code run on mode init or exit.

    //Overide these in you robot spec file if you need to insert code on mode changes
    // there are a few other cutpoints (code shims), add if needed.
    default public void teleopInit() { }
    default public void teleopExit() { }

    default public void periodic() { }

    default public void disabledExit() {}
    default public void disabledInit() {}
    default public void disabledPeriodic() {}
    
    default public void autonomousInit() {}
    default public void autonomousExit() {}
}
