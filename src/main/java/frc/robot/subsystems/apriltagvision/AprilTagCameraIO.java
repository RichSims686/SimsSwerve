package frc.robot.subsystems.apriltagvision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface AprilTagCameraIO {

    public class AprilTagCameraIOInputs implements LoggableInputs {
    
        public Transform3d robotToCamera;
        public boolean isConnected;
        public double timestamp;
        public Optional<Pose3d> visionPose;   // robot pose estimated from fixed target locations
        public List<AprilTagTarget> cameraToTargets = new ArrayList<>();    // list of all targets and their location relative to the robot
    
        // AdvantageKit's @AutoLog annotation and processInputs() function 
        // cannot handle Optional types,so we will manually replace
        // Optional.empty() <--> NaNs with our own toLog() and fromLog() functions

        @Override
        public final void toLog(LogTable table)
        {
            table.put("isConnected", isConnected);
            table.put("timestamp", timestamp);

            double[] data = new double[7];
            if (visionPose.isPresent()) {
                Pose3d pose = visionPose.get();
                data[0] = pose.getX();
                data[1] = pose.getY();
                data[2] = pose.getZ();
                data[3] = pose.getRotation().getQuaternion().getW();
                data[4] = pose.getRotation().getQuaternion().getX();
                data[5] = pose.getRotation().getQuaternion().getY();
                data[6] = pose.getRotation().getQuaternion().getZ();
            } else {
                for (int k=0; k<7; k++) {
                    data[k] = Double.NaN;
                }
            }
            table.put("visionPose", data);

            // TODO: add cameraToTarget list???
        }
    
        @Override
        public final void fromLog(LogTable table)
        {
            //2024isConnected = table.get("isConnected", false);
            isConnected = table.getBoolean("isConnected", false);
            double[] defaultData = {Double.NaN, Double.NaN, Double.NaN};
            //2024double[] data = table.get("visionPose", defaultData);
            //2024timestamp = table.get("timestamp", 0.0);
            double[] data = table.getDoubleArray("visionPose", defaultData);
            timestamp = table.getDouble("timestamp", 0.0);
    
            // convert double[] back to Pose3d
            if (Double.isNaN(data[0])) {
                visionPose = Optional.empty();
            } else {
                visionPose = Optional.of(new Pose3d(data[0], data[1], data[2], 
                    new Rotation3d(new Quaternion(data[3], data[4], data[5], data[6]))));
            }

            // TODO: add cameraToTarget list???
        }    
    }
    
    public default void updateInputs(AprilTagCameraIOInputs inputs) {}
    public default Optional<PhotonTrackedTarget> findDemoTag(int demoTagId) {return Optional.empty();}



    public class AprilTagTarget {
        private final int fiducialId;
        private final Transform3d cameraToTarget;
        private final double ambiguity;

        public AprilTagTarget(int fiducialId, Transform3d cameraToTarget, double ambiguity) {
            this.fiducialId = fiducialId;
            this.cameraToTarget = cameraToTarget;
            this.ambiguity = ambiguity;
        }

        public int getFiducialId() {
            return fiducialId;
        }

        public Transform3d getCameraToTarget() {
            return cameraToTarget;
        }

        public double getAmbiguity() {
            return ambiguity;
        }   
    }
}