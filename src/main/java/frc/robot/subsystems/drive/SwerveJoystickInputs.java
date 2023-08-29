package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.DriveConstants;

public class SwerveJoystickInputs 
  {
    double linearMagnitude;
    double linearAngleRadians;
    double turn;

    public SwerveJoystickInputs(double xIn, double yIn, double turnIn, boolean squareLinearInputs, boolean squareTurnInputs, boolean precisionEnable) {
      double x = applyDeadband(xIn);
      double y = applyDeadband(yIn);
      turn = applyDeadband(turnIn);

      linearMagnitude = Math.hypot(x, y);
      linearAngleRadians = Math.atan2(y, x);

      // apply non-lineariy for increased sensitivity for smaller movements
      if (squareLinearInputs) {
        linearMagnitude = linearMagnitude * linearMagnitude;
      }
      if (squareTurnInputs) {
        turn = Math.copySign(turn*turn, turn);
      }

      // limit to unit circle
      linearMagnitude = MathUtil.clamp(linearMagnitude, -1.0, +1.0);

      // Apply speed limits        
      if (precisionEnable) {
        linearMagnitude *= DriveConstants.precisionLinearMultiplier;
        turn *= DriveConstants.precisionTurnMulitiplier;
      }
    }

    
    private double applyDeadband(double in) {
      double out = 0;
      double deadband = DriveConstants.driveJoystickDeadbandPercent;
      if (Math.abs(in) > deadband) {
        out = Math.copySign((Math.abs(in) - deadband) / (1 - deadband), in);
      }
      return out;
    }

    public double getX() { return linearMagnitude * Math.cos(linearAngleRadians); }
    public double getY() { return linearMagnitude * Math.sin(linearAngleRadians); }
    public double getTurn() { return turn; }
  }