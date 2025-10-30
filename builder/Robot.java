// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.builder;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.Comparator;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Use this version of Robot with the library.
// RobotSpec_<year> can supply added code for any transition point
// See teleopInit() for example.

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private IRobotSpec m_IRobotSpec;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_IRobotSpec = RobotContainer.getRobotSpecs();
  }

  @Override
  public void robotPeriodic() {
    // call our robot spec's periodic
    m_IRobotSpec.periodic();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_IRobotSpec.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    m_IRobotSpec.disabledPeriodic();
  }

  @Override
  public void disabledExit() {
    m_IRobotSpec.disabledExit();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    m_IRobotSpec.autonomousExit();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // call any robot spec'd teleOpInit needed by the bot.
    m_IRobotSpec.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
    m_IRobotSpec.teleopExit();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  // Copies are made to move specific robot's deploy folder to root location.
  // For example deploy/2025 gets 2025 folder contents copied one dir up.
  // deploy/2025/pathplanner --> deploy/pathplanner.
  // The working pathplanner folder is deleted before the copy.
  static void copyFiles(String robot_deploy_dir){
    Path rootDir = Filesystem.getDeployDirectory().toPath();
    Path sourceDir = Paths.get( rootDir + robot_deploy_dir); // robot specific 
    
    // clean up working rootDir so the working dir's pathplanner is not poluted (important for sim)
    deleteDirectory(Paths.get(rootDir + "/pathplanner"));

    System.out.println("Copying files from ." + robot_deploy_dir + " to " + rootDir);
    // Traverse the tree, copy each file/directory from specific Robot deploy, e.g. chadbot, 2024, 2025...
    try {
      Files.walk(sourceDir)
        .filter(p -> !p.equals(sourceDir)) //prevent IOEx on first target
        .forEach(sourcePath -> {
           try {
              Path targetPath = rootDir.resolve(sourceDir.relativize(sourcePath));
              //System.out.printf("\tCopying %s to %s%n", sourcePath, targetPath);
              Files.copy(sourcePath, targetPath, StandardCopyOption.REPLACE_EXISTING);
           } catch (IOException ex) {
               System.out.format("I/O error: %s%n", ex);
           }
        });
    } catch (IOException e) {
      System.out.format("Error in copy %s%n", e.getMessage());
    }    
    System.out.println("Copy complete.");
  }

  static void deleteDirectory(Path del_folder) {
    System.out.format("Deleting working folder %s, replacing contents with Robot's spec'd %n", del_folder);
    try (var dirStream = Files.walk(del_folder)) {
      // Sort in reverse order to delete children before parents
      dirStream.sorted(Comparator.reverseOrder()) 
        .forEach(path -> {
            try {
              Files.delete(path);
            } catch (IOException e) {               
              System.err.println("Failed to delete: " + path + " - " + e.getMessage());
            }
        });
    }
    catch (IOException e){
     //don't care if folder was already deleted
    }
  }

}
