package frc.lib2202.subsystem.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IHeadingProvider {
    public Rotation2d getRotation2d();
    public void setRotation2d(Rotation2d heading);

    // alt names
    default public void setHeading(Rotation2d gyro_heading) { setRotation2d(gyro_heading); }
    default public Rotation2d getHeading() { return getRotation2d(); }
}
