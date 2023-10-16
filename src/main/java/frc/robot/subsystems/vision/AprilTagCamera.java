package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.AprilTagCameraIO.AprilTagCameraIOInputs;

public class AprilTagCamera {

    private final String name;
    private final AprilTagCameraIO cameraIO;
    private final AprilTagCameraIOInputs inputs = new AprilTagCameraIOInputs();

    // error check variables
    private double prevTimestamp = -1.0;
    private final double poseZThresh = 0.5; // meters, poses should be at least this close to the floor



    public AprilTagCamera(String name, AprilTagCameraIO cameraIO) {
        this.name = name;
        this.cameraIO = cameraIO;
    }

    public void periodic() {
        cameraIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Vision/Camera/" + name, inputs);

        if (inputs.visionPose.isEmpty()) {
            return;
        }
        Pose3d visionPose3d = inputs.visionPose.get();

        // do not update robot state if this is not a fresh vision estimate
        if (this.prevTimestamp == inputs.timestamp) {
            return;
        }
        prevTimestamp = inputs.timestamp;

        // filter out any vision readings that place the robot more than a certain distance from the floor
        if (Math.abs(visionPose3d.getZ()) > poseZThresh) {
            return;
        }
        Pose2d visionPose2d = visionPose3d.toPose2d();

        // filter out any vision readings that place the robot outside the field
        if ((visionPose2d.getX() < 0.0) || (visionPose2d.getX() > FieldConstants.fieldLength) || 
            (visionPose2d.getY() < 0.0) || (visionPose2d.getY() > FieldConstants.fieldWidth)) {
            return;
        }

        // Logger.getInstance().recordOutput("Vision/UpdateOdometry/Camera" + name, visionPose2d);

        // calculate robot to camera distance
        Pose2d robotPose2d = RobotState.getInstance().getPose();
        double distanceMeters = visionPose2d.relativeTo(robotPose2d).getTranslation().getNorm();
        
        // update RobotState
        RobotState.getInstance().addVisionMeasurement(
            visionPose2d,
            computeStdDevs(distanceMeters), 
            inputs.timestamp);
    }

    private Matrix<N3, N1> computeStdDevs(double distance) {

        double xyStdDev = VisionConstants.k1XYStdDev * distance*distance + VisionConstants.k0XYStdDev;
        double headingStdDev = VisionConstants.k1HeadingStdDev * distance*distance + VisionConstants.k0HeadingStdDev;
        return VecBuilder.fill(xyStdDev, xyStdDev, headingStdDev);
    }
    
}