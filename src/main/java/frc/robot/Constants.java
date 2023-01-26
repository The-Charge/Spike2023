// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot;

import java.util.Collections;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose3d;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    public static final class DriveConstants {
        public static final double kTrackwidthMeters = .609; //.4826; 
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
        public static final double kWheelDiameterMeters = 0.152;
        //Need to adjust, values are from example
        public final static double LINEAR_P = 0.1;
        public final static double LINEAR_D = 0.0;
        public final static double ANGULAR_P = 0.1;
        public final static double ANGULAR_D = 0.0;
    }
    public static class VisionConstants {
        public static final String frontCameraName = "Logi_Webcam_C920e"; //Name for Logitech Camera
        public static final String tapeCameraName = "HD_Webcam_C615";

        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(0); //Height of camera on robot
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    }
}

