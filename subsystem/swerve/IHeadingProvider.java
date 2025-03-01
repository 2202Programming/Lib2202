package frc.lib2202.subsystem.swerve;

import static frc.lib2202.Constants.DEGperRAD;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IHeadingProvider {
    public Rotation2d getRotation2d();
    /*
     * sets the heading provider to the given heading
     */
    public void setRotation2d(Rotation2d heading);
    
    /*
     * Returns angular rate of heading gyro.  [deg/s]
     */
    public double getYawRate();

    // alt names - [rad/s] or Rotation2d objects
    default public void setHeading(Rotation2d gyro_heading) { setRotation2d(gyro_heading); }
    default public Rotation2d getHeading() { return getRotation2d(); }
    /*
     * Heading Rate in [rad/s]
     */
    default public double getHeadingRate() { return getYawRate()/DEGperRAD; }

}
