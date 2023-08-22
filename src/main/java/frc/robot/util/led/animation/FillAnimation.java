package frc.robot.util.led.animation;

import frc.robot.util.led.LEDColor;
import frc.robot.util.led.strips.LEDStrip;

public class FillAnimation extends LEDAnimation {
    private final LEDStrip[] strips;
    private final LEDColor color;
    
    public FillAnimation(LEDColor color, LEDStrip... strips) {
        this.strips = strips;
        this.color = color;
    }

    @Override
    protected void runAnimation(LEDManager manager) {
        for(LEDStrip ledStrip : strips) {
            ledStrip.foreach((int i) -> {
                ledStrip.setLED(i, color);
            });
        }
    }
}
