package frc.robot.util.led.animation;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.led.LEDColor.RGBColor;
import frc.robot.util.led.strips.LEDStrip;

public abstract class LEDAnimation {
    public int priority;
    public final int getPriority() {return priority;}
    public final void setPriority(int priority) {this.priority = priority;}

    public final Timer animationTimer = new Timer();
    
    public final boolean isRunning() {return isRunning(LEDManager.getInstance());}
    public final boolean isRunning(LEDManager manager) {return manager.isAnimationRunning(this);}

    public final boolean start() {return start(LEDManager.getInstance());}
    public final boolean start(LEDManager manager) {
        return manager.play(this);
    }

    public final boolean pause() {return pause(LEDManager.getInstance());}
    public final boolean pause(LEDManager manager) {
        return manager.pause(this);
    }

    public final boolean stop() {return stop(LEDManager.getInstance());}
    public final boolean stop(LEDManager manager) {
        return manager.stop(this);
    }

    protected abstract void runAnimation(LEDManager manager);

    public static class StripCounterAnimation extends LEDAnimation {
        private final LEDStrip[] strips;

        public StripCounterAnimation(LEDStrip... strips) {
            this.strips = strips;
        }

        @Override
        protected void runAnimation(LEDManager manager) {
            for(LEDStrip ledStrip : strips) {
                for(int i = 0; i < ledStrip.getLength(); i++) {
                    int ind = i + 1;
                    if(ind % 50 == 0) {
                        ledStrip.setLED(i, new RGBColor(0, 255, 0));
                    } else if(ind % 10 == 0) {
                        ledStrip.setLED(i, new RGBColor(255, 0, 0));
                    } else if(ind % 5 == 0) {
                        ledStrip.setLED(i, new RGBColor(0, 0, 255));
                    } else {
                        ledStrip.setLED(i, new RGBColor(0, 0, 0));
                    }
                }
            }
        }

    }
}
