package frc.lib2202.subsystem.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.util.PIDFController;

/*
 * ISwerveModule - what SwerveDrivetrain needs from a single swerve module.
 *
 * Implementations:
 *   SwerveModuleMK3     - REV SparkMax / SparkFlex (Neo, Vortex)
 *   SwerveModuleTalonFX - CTRE TalonFX (Kraken, Falcon)
 *
 * Units: angles [deg], velocity [m/s], position [m]
 */
public interface ISwerveModule {

  // read sensors, called once per robot loop
  void periodic();

  void simulationPeriodic();

  void setDesiredState(SwerveModuleState state);

  // controlled wheel angle, unbounded [deg]
  double getAngle();

  Rotation2d getAngleRot2d();

  // absolute encoder angle, +/-180 [deg]
  double getAngleExternal();

  // wheel velocity [m/s]
  double getVelocity();

  // wheel position [m]
  double getPosition();

  SwerveModulePosition getSMPosition();

  void setDrivePosition(double position);

  // PID accessors for use in Test/Tune Commands
  void setDrivePID(PIDFController temp);

  void setAnglePID(PIDFController temp);

  void setBrakeMode();

  void setCoastMode();

  Command getWatcher();
}
