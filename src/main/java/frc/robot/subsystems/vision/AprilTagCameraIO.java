package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;

public interface AprilTagCameraIO {

    public class AprilTagCameraIOInputs implements LoggableInputs {
    
        public boolean isConnected;
        public Optional<Pose3d> visionPose;   
        public double timestamp;
    
        // AdvantageKit's @AutoLog annotation and processInputs() function 
        // cannot handle Optional types,so we will manually replace
        // Optional.empty() <--> NaNs with our own toLog() and fromLog() functions

        @Override
        public final void toLog(LogTable table)
        {
            table.put("isConnected", isConnected);
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
            table.put("timestamp", timestamp);
        }
    
        @Override
        public final void fromLog(LogTable table)
        {
            isConnected = table.getBoolean("isConnected", false);
            double[] defaultData = {Double.NaN, Double.NaN, Double.NaN};
            double[] data = table.getDoubleArray("visionPose", defaultData);
            timestamp = table.getDouble("timestamp", 0.0);
    
            // convert double[] back to Pose3d
            if (Double.isNaN(data[0])) {
                visionPose = Optional.empty();
            } else {
                visionPose = Optional.of(new Pose3d(data[0], data[1], data[2], 
                    new Rotation3d(new Quaternion(data[3], data[4], data[5], data[6]))));
            }
        }    
    }
    
    public default void updateInputs(AprilTagCameraIOInputs inputs) {}

}
