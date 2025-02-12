package frc.lib2202.subsystem.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib2202.subsystem.OdometryInterface;

public class AutoPPConfigure {

    static RobotConfig GUIconfig;

    // AutoBuilder for PathPlanner - uses internal static vars in AutoBuilder
    public static void configureAutoBuilder(DriveTrainInterface sdt, OdometryInterface odometry) {
        // use default pids for trans/rot
        configureAutoBuilder(sdt, odometry,
                new PIDConstants(7.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(7.0, 0.0, 0.0)); // Rotation PID constants
    }

    public static void configureAutoBuilder(DriveTrainInterface sdt, OdometryInterface odometry,
            PIDConstants translationPID, PIDConstants rotPID) {
        try {
            GUIconfig = RobotConfig.fromGUISettings();

            // Configure the AutoBuilder last
            AutoBuilder.configure(
                    odometry::getPose, // Robot pose supplier
                    odometry::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    sdt::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    sdt::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                    new PPHolonomicDriveController( // HolonomicPathFollowerConfig,                                                   
                            translationPID, // Translation PID constants
                            rotPID), // Rotation PID constants
                    GUIconfig,
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    sdt // required sub-systems for driving
            );

        } catch (Exception e) {
            // Handle exception as needed
            System.out
                    .println("PATHING - Could not initialize PathPlanner check for ~/deploy/pathplanner/settings.json");
            e.printStackTrace();
            System.out.println("PATHING - End of stack trace --------------");
        }

    }

}
