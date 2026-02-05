package frc.robot2026.util;

import edu.wpi.first.math.geometry.Transform3d;


public class PhotonvisionConfig {
    // photonvision camera names (needs to match photonvision UI naming)
    public String[] CAMERA_NAMES;

    // Robot to camera transforms.
    // https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html
    public Transform3d[] kRobotToCam;

    public PhotonvisionConfig(String[] CAMERA_NAMES, Transform3d[] kRobotToCam){
        this.CAMERA_NAMES = CAMERA_NAMES;
        this.kRobotToCam = kRobotToCam;
    }

}
