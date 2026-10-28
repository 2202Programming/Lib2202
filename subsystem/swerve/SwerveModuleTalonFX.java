package frc.lib2202.subsystem.swerve;

import static frc.lib2202.Constants.DT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.Robot;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.util.ModMath;
import frc.lib2202.util.PIDFController;

/**
 * SwerveModuleTalonFX - TalonFX (Kraken / Falcon) version of SwerveModuleMK3.
 *
 * Drive motor uses VelocityVoltage. SensorToMechanismRatio is set so the
 * TalonFX "mechanism" units are wheel meters, so velocity is [m/s] and
 * position is [m], same as the Spark version.
 *
 * Angle motor uses PositionVoltage with the CANcoder as a RemoteCANcoder
 * feedback sensor. Mechanism units are wheel rotations. Because the TalonFX
 * closes the loop on the CANcoder directly, no calibrate() step is needed at
 * power-up. The CANcoder position is continuous (unbounded), so we use the same
 * unbounded delta-angle approach as MK3.
 *
 * Angle direction comes from the CANcoder (CCW positive, set in
 * SwerveDrivetrain.initCANcoder()), so ModuleConfig.kAngleCmdInvert is not
 * used. Set kAngleMotorInvert so positive motor voltage turns the wheel CCW.
 *
 * PIDF units are CTRE's, not REV's:
 *   drivePIDF - kP [V per m/s error], kF -> kV [V per m/s]
 *   anglePIDF - kP [V per wheel-rotation error], kF -> kV [V per rot/s]  (CTRE examples use something like 50–100)
 * so chassis gains from a Spark robot will need re-tuning.
 */
public class SwerveModuleTalonFX implements ISwerveModule {
  public final String NT_Name = "DriveModules";
  static final int kRetries = 5;

  // Chassis config used for geometry and pathing math
  private final ChassisConfig cc;

  // CTRE devices
  private final TalonFX driveMotor;
  private final TalonFX angleMotor;
  private final CANcoder absEncoder;
  private final TalonFXConfiguration driveCfg;
  private final TalonFXConfiguration angleCfg;
  private final double wheelFactor; // factor used to account for wheelwear
  private final double driveSensorToMech; // [mo-rot / m]

  // control requests, reused every cycle
  private final VelocityVoltage driveRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage angleRequest = new PositionVoltage(0).withSlot(0);

  // status signals, refreshed together in periodic()
  private final StatusSignal<Angle> drivePosSig;
  private final StatusSignal<AngularVelocity> driveVelSig;
  private final StatusSignal<Angle> anglePosSig;
  private final StatusSignal<Angle> absPosSig;

  // NetworkTables
  String NTPrefix;
  public String myprefix;

  // m_ -> measurements made every period, same meaning as SwerveModuleMK3
  double m_internalAngle; // measured CANcoder via TalonFX, unbounded [deg]
  double m_internalAngleMod; // modulo
  double m_externalAngle; // measured CANCoder absolute, bounded +/-180 [deg]
  double m_velocity; // measured velocity [m/s]
  double m_position; // measure wheel positon [m]
  double m_angle_target; // desired angle bounded, from state [deg]
  double m_angle_delta; // delta angle being commanded [deg]
  double m_angle_cmd; // unbounded angle sent to TalonFX [deg]
  double m_vel_target; // desired velocity [m/s]

  public SwerveModuleTalonFX(TalonFX driveMtr, TalonFX angleMtr, CANcoder absEnc,
      boolean invertAngleMtr, boolean invertDrive, String prefix, double wheelFactor) {
    driveMotor = driveMtr;
    angleMotor = angleMtr;
    absEncoder = absEnc;
    myprefix = prefix;
    this.wheelFactor = wheelFactor;

    IRobotSpec specs = RobotContainer.getRobotSpecs();
    RobotLimits limits = specs.getRobotLimits();
    cc = specs.getChassisConfig();

    // Drive Motor config - mechanism units are wheel meters
    driveSensorToMech = cc.kDriveGR / (wheelFactor * Math.PI * cc.wheelDiameter); // [mo-rot/m]
    driveCfg = new TalonFXConfiguration();
    driveCfg.MotorOutput
        .withInverted(invertDrive ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    driveCfg.CurrentLimits
        .withStatorCurrentLimit(limits.driveStallAmp)
        .withStatorCurrentLimitEnable(true);
    driveCfg.Feedback
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(driveSensorToMech);
    copyPIDF(cc.drivePIDF, driveCfg.Slot0);
    applyConfig(driveMotor, driveCfg, "drive");
    setDrivePosition(0.0);

    // Angle Motor config - closes loop on CANcoder, mechanism units are wheel rotations
    angleCfg = new TalonFXConfiguration();
    angleCfg.MotorOutput
        .withInverted(invertAngleMtr ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    angleCfg.CurrentLimits
        .withStatorCurrentLimit(limits.angleStallAmp)
        .withStatorCurrentLimitEnable(true);
    angleCfg.Feedback
        .withFeedbackRemoteSensorID(absEncoder.getDeviceID())
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withRotorToSensorRatio(cc.kSteeringGR)
        .withSensorToMechanismRatio(1.0);
    copyPIDF(cc.anglePIDF, angleCfg.Slot0);
    applyConfig(angleMotor, angleCfg, "angle");

    drivePosSig = driveMotor.getPosition();
    driveVelSig = driveMotor.getVelocity();
    anglePosSig = angleMotor.getPosition();
    absPosSig = absEncoder.getAbsolutePosition();

    NTPrefix = "FX-" + myprefix + " CAN(ang=" + angleMtr.getDeviceID() +
        ", drv=" + driveMtr.getDeviceID() + ")";

    simulationInit(specs);
    periodic();
    m_angle_cmd = m_internalAngle; // hold current angle until commanded
    getWatcher(); // no longer auto start watchers
  }

  // copy our PIDF into a CTRE slot, F is used as the velocity feed forward kV
  static void copyPIDF(PIDFController pidf, Slot0Configs slot) {
    slot.withKP(pidf.getP())
        .withKI(pidf.getI())
        .withKD(pidf.getD())
        .withKV(pidf.getF());
  }

  // Retry config apply, report if failure
  void applyConfig(TalonFX motor, TalonFXConfiguration cfg, String name) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < kRetries; ++i) {
      status = motor.getConfigurator().apply(cfg, 1.0); // power up we can wait
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("*** ERROR *** " + myprefix + " " + name + " TalonFX(" + motor.getDeviceID() +
          ") config failed: " + status.toString());
    }
  }

  // PID accessor for use in Test/Tune Commands
  public void setDrivePID(PIDFController temp) {
    copyPIDF(temp, driveCfg.Slot0);
    driveMotor.getConfigurator().apply(driveCfg.Slot0);
  }

  public void setAnglePID(PIDFController temp) {
    copyPIDF(temp, angleCfg.Slot0);
    angleMotor.getConfigurator().apply(angleCfg.Slot0);
  }

  public String getNTPrefix() {
    return NTPrefix;
  }

  public void periodic() {
    // measure everything at same time; these get updated every cycle
    BaseStatusSignal.refreshAll(drivePosSig, driveVelSig, anglePosSig, absPosSig);
    m_internalAngle = anglePosSig.getValueAsDouble() * 360.0;
    m_internalAngleMod = ModMath.fmod360(m_internalAngle);
    m_velocity = driveVelSig.getValueAsDouble();
    m_position = drivePosSig.getValueAsDouble();
    m_externalAngle = absPosSig.getValueAsDouble() * 360.0;
  }

  // driveEncoder position [m]
  public void setDrivePosition(double position) {
    m_position = position;
    driveMotor.setPosition(position);
  }

  public Rotation2d getAngleRot2d() {
    return Rotation2d.fromDegrees(m_internalAngle);
  }

  public double getAngle() {
    return m_internalAngle;
  }

  public Rotation2d getAngleExternalRot2d() {
    return Rotation2d.fromDegrees(m_externalAngle);
  }

  public double getAngleExternal() {
    return m_externalAngle;
  }

  public double getVelocity() {
    return m_velocity;
  }

  public double getPosition() {
    return m_position;
  }

  public SwerveModulePosition getSMPosition() {
    return new SwerveModulePosition(m_position, getAngleRot2d());
  }

  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   *
   * @param desiredState - A SwerveModuleState representing the desired new state
   *                     of the module
   */
  public void setDesiredState(SwerveModuleState state) {
    // should favor reversing direction over turning > 90 degrees
    state.optimize(Rotation2d.fromDegrees(m_internalAngle));

    m_angle_target = state.angle.getDegrees();

    // figure out how far we need to move, target - current, bounded +/-180
    m_angle_delta = ModMath.delta360(m_angle_target, m_internalAngle);
    // if we aren't moving, keep the wheels pointed where they are
    m_angle_delta = (Math.abs(state.speedMetersPerSecond) < .01) ? 0.0 : m_angle_delta;

    // add delta to unbounded CANcoder angle, TalonFX wants rotations
    m_angle_cmd = m_internalAngle + m_angle_delta;
    angleMotor.setControl(angleRequest.withPosition(m_angle_cmd / 360.0));

    // save target vel for plots
    m_vel_target = state.speedMetersPerSecond;
    driveMotor.setControl(driveRequest.withVelocity(m_vel_target));
  }

  /*
   * Simulation stuff - ideal drive velocity, 1st order angle response, same as MK3
   */
  TalonFXSimState driveSim;
  CANcoderSimState simAbsEncoder;
  LinearFilter simAngleActuator;
  double simMagOffset; // [rot] CANcoder sim raw position is before mag offset

  void simAngleActuator() {
    // only run when enabled
    if (!DriverStation.isEnabled()) return;
    double pos = m_internalAngle / 360.0;
    double new_pos = simAngleActuator.calculate(m_angle_cmd / 360.0); // [rot]
    simAbsEncoder.setRawPosition(new_pos - simMagOffset);
    simAbsEncoder.setVelocity((new_pos - pos) / DT);
  }

  public void simulationPeriodic() {
    simAngleActuator();
    // ideal drive, wheel tracks commanded velocity
    double vel = DriverStation.isEnabled() ? m_vel_target : 0.0; // [m/s]
    driveSim.setRotorVelocity(vel * driveSensorToMech);
    driveSim.addRotorPosition(vel * driveSensorToMech * DT);
  }

  public void simulationInit(IRobotSpec spec) {
    if (!Robot.isSimulation()) return; // skip sim on real
    driveSim = driveMotor.getSimState();
    driveSim.Orientation = (driveCfg.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
        ? ChassisReference.Clockwise_Positive
        : ChassisReference.CounterClockwise_Positive;
    simAbsEncoder = absEncoder.getSimState();

    MagnetSensorConfigs mag = new MagnetSensorConfigs();
    absEncoder.getConfigurator().refresh(mag);
    simMagOffset = mag.MagnetOffset;

    simAngleActuator = LinearFilter.singlePoleIIR(.025, DT);
    double[] simActInit = { anglePosSig.refresh().getValueAsDouble() };
    simAngleActuator.reset(simActInit, simActInit);
  }

  public class SwerveModuleTalonFXWatcher extends WatcherCmd {
    private NetworkTableEntry nte_angle;
    private NetworkTableEntry nte_angleMod;
    private NetworkTableEntry nte_delta;
    private NetworkTableEntry nte_external_angle;
    private NetworkTableEntry nte_velocity;
    private NetworkTableEntry nte_position;
    private NetworkTableEntry nte_angle_target;
    private NetworkTableEntry nte_vel_target;
    private NetworkTableEntry nte_motor_current;
    private NetworkTableEntry nte_applied_output;

    @Override
    public String getTableName() {
      return NT_Name + "/" + NTPrefix;
    }

    @Override
    public void ntcreate() {
      NetworkTable table = getTable();
      nte_angle = table.getEntry("m_internalAngle");
      nte_delta = table.getEntry("m_angle_delta");
      nte_angleMod = table.getEntry("m_internalAngleMod");
      nte_external_angle = table.getEntry("m_externalAngle");
      nte_angle_target = table.getEntry("m_angle_target");
      nte_vel_target = table.getEntry("whl velocity_target");
      nte_velocity = table.getEntry("whl velocity");
      nte_position = table.getEntry("whl position");
      nte_motor_current = table.getEntry("whl mtr current");
      nte_applied_output = table.getEntry("whl mtr apply_out");
    }

    @Override
    public void ntupdate() {
      nte_angle.setDouble(fmt2(m_internalAngle));
      nte_delta.setDouble(fmt2(m_angle_delta));
      nte_angleMod.setDouble(fmt2(m_internalAngleMod));
      nte_external_angle.setDouble(fmt2(m_externalAngle));
      nte_velocity.setDouble(fmt2(m_velocity));
      nte_position.setDouble(fmt2(m_position));
      nte_angle_target.setDouble(fmt2(m_angle_target));
      nte_vel_target.setDouble(fmt2(m_vel_target));
      nte_motor_current.setDouble(fmt2(driveMotor.getStatorCurrent().getValueAsDouble()));
      nte_applied_output.setDouble(fmt2(driveMotor.getDutyCycle().getValueAsDouble()));
    }
  } // watcher class

  // access the watcher
  public Command getWatcher() {
    return this.new SwerveModuleTalonFXWatcher();
  }

  public void setBrakeMode() {
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    angleMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setCoastMode() {
    driveMotor.setNeutralMode(NeutralModeValue.Coast);
    angleMotor.setNeutralMode(NeutralModeValue.Coast);
  }
}
