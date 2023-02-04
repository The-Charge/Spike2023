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
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        //Need to be tuned
        public final static double LINEAR_P = 1.5; //0.5 bad. 1 ok. 1.5 ok. 2 is pretty good
        public final static double LINEAR_D = 0.0;
        // public final static double ANGULAR_P = 0.002;
        public final static double ANGULAR_P = 0.05; // > 0.2 is no bueno < 0.02 is muy no bueno 0.025 is maybe
        public final static double ANGULAR_D = 0.0;
    }
    public static class VisionConstants {
        public static final String frontCameraName = "Logi_Webcam_C920e"; //Name for Logitech Camera
        public static final String tapeCameraName = "HD_Webcam_C615";

        public static final double TAGCAMERA_HEIGHT_METERS = Units.inchesToMeters(8); //Height of camera on robot
        public static final double TAGCAMERA_PITCH_RADIANS = Units.degreesToRadians(10); //VERY IMPORTANT
    
        public static final double TAPECAMERA_HEIGHT_METERS = Units.inchesToMeters(7.5); //Height of camera on robot
        public static final double TAPECAMERA_PITCH_RADIANS = Units.degreesToRadians(20); 
    }
}

