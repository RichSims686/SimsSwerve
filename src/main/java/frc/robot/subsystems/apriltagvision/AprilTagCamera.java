package frc.robot.subsystems.apriltagvision;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.apriltagvision.AprilTagCameraIO.AprilTagCameraIOInputs;

public class AprilTagCamera {

    private final String name;
    private final AprilTagCameraIO cameraIO;
    private final AprilTagCameraIOInputs inputs = new AprilTagCameraIOInputs();

    // error check variables
    private double prevTimestamp = -1.0;
    private static final double fieldXYMargin = 0.5;
    private static final double fieldZMargin = 0.5; // meters, poses should be at least this close to the floor
  
    private boolean visionUpdatesEnabled = true;


    public AprilTagCamera(String name, AprilTagCameraIO cameraIO) {
        this.name = name;
        this.cameraIO = cameraIO;
    }

    public void periodic() {
        cameraIO.updateInputs(inputs);
        Logger.getInstance().processInputs("AprilTagVision/Camera/" + name, inputs);

        if (inputs.visionPose.isEmpty()) {
            return;
        }
        Pose3d visionPose3d = inputs.visionPose.get();

        // do not update robot state if this is not a fresh vision estimate
        if (this.prevTimestamp == inputs.timestamp) {
            return;
        }
        prevTimestamp = inputs.timestamp;

        // filter out any vision readings that place the robot outside the field
        if (visionPose3d.getX() < -fieldXYMargin
            || visionPose3d.getX() > FieldConstants.fieldLength + fieldXYMargin
            || visionPose3d.getY() < -fieldXYMargin
            || visionPose3d.getY() > FieldConstants.fieldWidth + fieldXYMargin
            || visionPose3d.getZ() < -fieldZMargin
            || visionPose3d.getZ() > +fieldZMargin) {
            return;
        }
        Pose2d visionPose2d = visionPose3d.toPose2d();

        // calculate robot to camera distance
        Pose2d robotPose2d = RobotState.getInstance().getPose();
        double distanceMeters = visionPose2d.relativeTo(robotPose2d).getTranslation().getNorm();
        
        if (visionUpdatesEnabled) {
            // update RobotState
            RobotState.getInstance().addVisionMeasurement(
                visionPose2d,
                computeStdDevs(distanceMeters), 
                inputs.timestamp);
        }

        Logger.getInstance().recordOutput("AprilTagVision/Camera/" + name + "/VisionPose2d", visionPose2d);
    }

    private Matrix<N3, N1> computeStdDevs(double distance) {
        double xyStdDev = VisionConstants.k1XYStdDev * distance*distance + VisionConstants.k0XYStdDev;
        double k0HeadingStdDev = DriverStation.isEnabled() ? VisionConstants.k0HeadingStdDev : VisionConstants.k0HeadingStdDevDisabled;
        double headingStdDev = VisionConstants.k1HeadingStdDev * distance*distance + k0HeadingStdDev;
        return VecBuilder.fill(xyStdDev, xyStdDev, headingStdDev);
    }
 
    public void setVisionUpdatesEnabled(boolean enable) {
        visionUpdatesEnabled = enable;
    }

    public Transform3d getRobotToCamera() {
        return inputs.robotToCamera;
    }

    public List<AprilTagCameraIO.AprilTagTarget> getTargetList() {
        return inputs.cameraToTargets;
    }
}