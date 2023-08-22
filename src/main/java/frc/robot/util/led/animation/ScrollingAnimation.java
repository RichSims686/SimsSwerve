package frc.robot.util.led.animation;

import frc.robot.util.led.functions.Gradient;
import frc.robot.util.led.functions.TilingFunction;
import frc.robot.util.led.strips.LEDStrip;

public class ScrollingAnimation extends LEDAnimation {
    private final LEDStrip[] strips;
    private final Gradient gradient;
    private final TilingFunction tilingFunction;

    private double              velocity = 1;
    public double               getVelocity()                   {return velocity;}
    public ScrollingAnimation    setVelocity(double velocity)    {this.velocity = velocity; return this;}

    private double              wavelength = 1;
    public double               getWavelength()                     {return wavelength;}
    public ScrollingAnimation    setWavelength(double wavelength)    {this.wavelength = wavelength; return this;}

    public ScrollingAnimation(Gradient gradient, LEDStrip... strips) {
        this(gradient, TilingFunction.Modulo, strips);
    }
    public ScrollingAnimation(Gradient gradient, TilingFunction tilingFunction, LEDStrip... strips) {
        this.strips = strips;
        this.gradient = gradient;
        this.tilingFunction = tilingFunction;
    }

    @Override
    protected void runAnimation(LEDManager manager) {
        for(LEDStrip ledStrip : strips) {
            ledStrip.foreach((int i) -> {
                double pos = (double) i / ledStrip.getLength();
                ledStrip.setLED(i, gradient.getColor(tilingFunction.tile(pos*wavelength - animationTimer.get()*velocity)));
            });
        }
    }
}
