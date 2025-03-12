// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.lib2202.subsystem.swerve;

import static frc.lib2202.Constants.DEGperRAD;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
//wip import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.util.ModMath;

public class SwerveDrivetrain extends DriveTrainInterface {
  static final String canBusName = "rio";
  static final double longWaitSeconds = 1.0; // cancode config wait

  // cc is the chassis config for all our pathing math
  // final RobotLimits limits
  final ChassisConfig cc; // from robotSpecs
  final ModuleConfig mc[]; // from robotSpecs

  /**
   *
   * Modules are in the order of - Front Left, Front Right, Back Left, Back Right
   * 
   * Positive x --> represent moving toward the front of the robot [m]
   * Positive y --> represent moving toward the left of the robot [m]
   *
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  final SwerveDriveKinematics kinematics;
  //final SwerveDriveOdometry m_odometry;
  //Pose2d m_pose; // pose based strictly on the odometry

  // controls behavior of visionPposeestimator
  boolean visionPoseUsingRotation = true;
  boolean visionPoseEnabled = true;

  // Swerver States and positions
  SwerveModuleState[] meas_states; // measured wheel speed & angle
  SwerveModulePosition[] meas_pos; // distance & angle for each module
  ChassisSpeeds speedsRC;
 
  // sensors and our mk3 modules
  final IHeadingProvider sensors;
  final SwerveModuleMK3[] modules;
  final CANcoder canCoders[];

  public SwerveDrivetrain() {
      this(SparkMax.class);
  }

  @SuppressWarnings("rawtypes")
  public SwerveDrivetrain(Class mtrClass) {
    cc = RobotContainer.getRobotSpecs().getChassisConfig();
    mc = RobotContainer.getRobotSpecs().getModuleConfigs();

    // Coords checked: Left +Y offset, right -Y offset, +X, front -x back.
    // See:
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    // match order to ModuleConfig[] in RobotSpec_<robot name>.java
    kinematics = new SwerveDriveKinematics(
        new Translation2d(cc.XwheelOffset, cc.YwheelOffset), // Front Left
        new Translation2d(cc.XwheelOffset, -cc.YwheelOffset), // Front Right
        new Translation2d(-cc.XwheelOffset, cc.YwheelOffset), // Back Left
        new Translation2d(-cc.XwheelOffset, -cc.YwheelOffset) // Back Right
    );

    // allocate space for measured positions, initialized to zeros
    meas_pos = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(),
        new SwerveModulePosition(), new SwerveModulePosition()
    };

    sensors = RobotContainer.getRobotSpecs().getHeadingProvider();
    canCoders = new CANcoder[mc.length];
    modules = new SwerveModuleMK3[mc.length];

    // create cancoders and swerve modules
    for (int i = 0; i < mc.length; i++) {
      canCoders[i] = initCANcoder(mc[i].CANCODER_ID, mc[i].kAngleOffset);

      modules[i] = new SwerveModuleMK3(
          mtrClass,
          // handle either flex or max for modules
          (mtrClass == SparkMax.class) ? new SparkMax(mc[i].DRIVE_MOTOR_ID, MotorType.kBrushless) : 
                                         new SparkFlex(mc[i].DRIVE_MOTOR_ID, MotorType.kBrushless),
          (mtrClass == SparkMax.class) ? new SparkMax(mc[i].ANGLE_MOTOR_ID, MotorType.kBrushless)  :
                                         new SparkFlex(mc[i].ANGLE_MOTOR_ID, MotorType.kBrushless),

          canCoders[i],
          mc[i].kAngleMotorInvert,
          mc[i].kAngleCmdInvert,
          mc[i].kDriveMotorInvert,
          mc[i].id.toString());

      /* Speed up signals to an appropriate rate */
      // long term wip BaseStatusSignal.setUpdateFrequencyForAll(100,
      // canCoders[i].getPosition(), canCoders[i].getVelocity());
    }

    speedsRC = new ChassisSpeeds(0, 0, 0);
    meas_states = kinematics.toSwerveModuleStates(speedsRC);
    offsetDebug();
    getWatcher();
  }

  /**
   * initCANcoder() - setup cancoder the way we need them.
   * This CANcoder returns value in rotation with phoenix 6 [-0.5, 0.5)
   * rotations (clock wise is positive).
   * canBusName set to "rio" at top of module
   * 
   * @param cc_ID
   * @param cc_offset_deg [+/- 180 deg]
   * 
   * @return CANcoder just initialized
   */
  private CANcoder initCANcoder(int cc_ID, double cc_offset_deg) {
    CANcoder canCoder = new CANcoder(cc_ID, canBusName);
    StatusSignal<Angle> abspos = canCoder.getAbsolutePosition().waitForUpdate(longWaitSeconds, true);
    StatusSignal<Angle> pos = canCoder.getPosition().waitForUpdate(longWaitSeconds, true);
    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; //AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    configs.MagnetSensor.MagnetOffset = cc_offset_deg / 360.0; // put offset deg on +/- 0.5 range
    configs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoder.clearStickyFaults(longWaitSeconds);

    // update mag offset and check status, report errors
    StatusCode status = canCoder.getConfigurator().apply(configs, longWaitSeconds);
    if (!status.isOK()) {
      System.out.println("Warning CANCoder(" + cc_ID + ") returned " +
          status.toString() + " on applying confg. Retrying");
      SwerveModuleMK3.sleep(100);
      canCoder.clearStickyFaults(longWaitSeconds);
      status = canCoder.getConfigurator().apply(configs, longWaitSeconds);
      System.out.println("CANCoder(" + cc_ID + ") status on retry: " + status.toString() + " moving on, good luck.");
    }
    canCoder.clearStickyFaults(longWaitSeconds);

    // Re-read sensor, blocking calls
    abspos.waitForUpdate(longWaitSeconds, true);
    pos.waitForUpdate(longWaitSeconds, true);
    /*
     * System.out.println("CC(" + cc_ID + ")  after: "+
     * "\tabspos = " + abspos.getValue() + " (" + abspos.getValue()*360.0+" deg)" +
     * "\tpos = " + pos.getValue() + " (" + pos.getValue()*360.0 + " deg)" );
     */
    return canCoder;
  }

  // debugging print for mag-offsets of canCoders
  private void offsetDebug() {
    periodic(); // run to initialize module values
    System.out.println("================OffsetDebug==================");
    for (int i = 0; i < mc.length; i++) {
      double offset = mc[i].kAngleOffset;
      double measured = modules[i].m_internalAngle;
      double cc_measured = modules[i].m_externalAngle;
      System.out.println(mc[i].id.toString() + ": offset=" + offset + ", internal=" + measured +
          " cc_meas=" + cc_measured + ", if zero-aligned, set mag offset = " +
          ModMath.fmod360_2(offset - cc_measured));
    }
    System.out.println("============OffsetDebug Done==============");
  }

  public void drive(SwerveModuleState[] states) {
    // if any one wheel is above max obtainable speed, reduce them all in the same
    // ratio to maintain control
    // SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrain.kMaxSpeed);

    // output the angle and velocity for each module
    for (int i = 0; i < states.length; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
  // used by pathPlaner
  public void driveRobotRelative(ChassisSpeeds chassisSpeed) {
    drive(kinematics.toSwerveModuleStates(chassisSpeed));
  }

  // used for testing
  @SuppressWarnings("unused")
  private void testDrive(double speed, double angle) {
    // output the angle and speed (meters per sec) for each module
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(new SwerveModuleState(speed, new Rotation2d(Math.toRadians(angle))));
    }
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return meas_pos;
  }

  @Override
  public void periodic() {
    // update data from each of the swerve drive modules.
    for (int i = 0; i < modules.length; i++) {
      modules[i].periodic();
      meas_states[i].speedMetersPerSecond = modules[i].getVelocity();
      meas_states[i].angle = meas_pos[i].angle = modules[i].getAngleRot2d();
      meas_pos[i].distanceMeters = modules[i].getPosition();
    }
    //chassis speeds 
    speedsRC = kinematics.toChassisSpeeds(meas_states);
  }

  static boolean simInit = false;
  // simple model for testing
  public void simulationInit() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].simulationInit();
    }
  }

  @Override
  public void simulationPeriodic() {
    if (!simInit) {
      simulationInit();
      simInit = true;
    }
    for (int i = 0; i < modules.length; i++) {
      modules[i].simulationPeriodic();
    }
  }

  //set all module positions to given position [m].
  // Used mostly for debugging pathing and other testing.
  // Note, odometry tracks wheel positions internally
  // so any position reset, should be used in conjuection 
  // with odometry.resetPosition() call.
  public void setPositions(double position) {
    for (int i = 0; i < modules.length; i++) {
      // module state/encoder
      modules[i].setPosition(position);
      // our local position copy
      meas_pos[i].distanceMeters = position;
    }
  }

  public SwerveModuleMK3 getMK3(int modID) {
    if ((modID < 0) || (modID > modules.length - 1))
      return null;
    return modules[modID];
  }


  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return speedsRC;
    //return kinematics.toChassisSpeeds(meas_states);
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    var speeds = getChassisSpeeds();
    var rot2d = sensors.getRotation2d();
    return new ChassisSpeeds(
      //avoid multiple calls to getCS() & sensors.getRotation2d()
      speeds.vxMetersPerSecond * rot2d.getCos() - speeds.vyMetersPerSecond * rot2d.getSin(),
      speeds.vyMetersPerSecond * rot2d.getCos() + speeds.vxMetersPerSecond * rot2d.getSin(),
      speeds.omegaRadiansPerSecond);
  }

  /**
   * stop() zero the current state's velocity component and leave angles
   */
  public void stop() {
    SwerveModuleState state = new SwerveModuleState();
    state.speedMetersPerSecond = 0.0;
    // output the angle and velocity for each module
    for (int i = 0; i < modules.length; i++) {
      state.angle = Rotation2d.fromDegrees(modules[i].getAngle());
      modules[i].setDesiredState(state);
    }
  }

  public void setBrakeMode() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setBrakeMode();
    }
    System.out.println("***BRAKES ENGAGED***");
  }

  public void setCoastMode() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setCoastMode();
    }
    System.out.println("***BRAKES RELEASED***");
  }

  Command getWatcher() {
    return this.new SwerveMonitorCmd();
  }

  /*
   * Watcher for SwerveDrivetrain and its vision data.
   *
   * Only watches high level data, for module details see the tables for each of
   * the modules.
   */
  public class SwerveMonitorCmd extends WatcherCmd {

    // chassis velocity
    NetworkTableEntry nt_radiansPerSecond;
    NetworkTableEntry nt_xMetersPerSec;
    NetworkTableEntry nt_yMetersPerSec;
    NetworkTableEntry nt_speeds;
    NetworkTableEntry nt_speedsDesc;

    NetworkTableEntry nt_deltas;
    
    public SwerveMonitorCmd() {

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
      nt_radiansPerSecond = MonitorTable.getEntry("Vrot");
      nt_xMetersPerSec = MonitorTable.getEntry("Vx ");
      nt_yMetersPerSec = MonitorTable.getEntry("Vy ");
      nt_speeds = MonitorTable.getEntry("speeds");
    }

    @Override
    public void ntupdate() {
      // robot coordinates - speeds
      nt_radiansPerSecond.setDouble(speedsRC.omegaRadiansPerSecond * DEGperRAD);
      nt_xMetersPerSec.setDouble(speedsRC.vxMetersPerSecond);
      nt_yMetersPerSec.setDouble(speedsRC.vyMetersPerSecond);
      nt_speeds.setString(speedsRC.toString());
    }
  }
}