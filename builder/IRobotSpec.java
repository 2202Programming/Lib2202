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

    default public void teleopInit() {
    }

    default public void periodic() {
    }

}
