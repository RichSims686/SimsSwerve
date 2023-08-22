package frc.robot.util.led.animation;

import frc.robot.util.led.LEDColor;
import frc.robot.util.led.functions.Gradient.BasicGradient;
import frc.robot.util.led.functions.Gradient.BasicGradient.InterpolationStyle;
import frc.robot.util.led.functions.TilingFunction;
import frc.robot.util.led.strips.LEDStrip;

public class EndgameNotificationAnim extends FlashingAnimation {
    public EndgameNotificationAnim(LEDStrip... strips) {
        super(new BasicGradient(InterpolationStyle.Linear, LEDColor.Black, LEDColor.Yellow), TilingFunction.Sawtooth, strips);
        setPeriod(0.25);
    }

    @Override
    protected void runAnimation(LEDManager manager) {
        if(animationTimer.hasElapsed(1)) {
            manager.stop(this);
            return;
        }
        super.runAnimation(manager);
    }
}
