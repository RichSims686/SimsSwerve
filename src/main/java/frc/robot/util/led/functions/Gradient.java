package frc.robot.util.led.functions;

import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.InterpolationFunction;
import frc.robot.util.led.LEDColor;
import frc.robot.util.led.LEDColor.HSVColor;
import frc.robot.util.led.LEDColor.RGBColor;

@FunctionalInterface
public interface Gradient {
    public LEDColor getColor(double x);

    public static class BasicGradient implements Gradient {
        private final LEDColor[] colors;
        private final InterpolationFunction<LEDColor> interpolationFunction;
        public static enum InterpolationStyle implements InterpolationFunction<LEDColor> {
            Step((double t, Double[] data) -> data[(int)MathUtil.clamp(Math.floor(t*data.length), 0, data.length-1)]),
            Linear((double t, Double[] data) -> {
                if(data.length <= 0) return 0.0;
                t *= (data.length - 1);
                int lower = (int)Math.floor(t);
                int upper = (int)Math.ceil(t);
                t = (t % 1 + 1) % 1;
                return (data[upper] - data[lower]) * t + data[lower];
            }),
            ;
            private final InterpolationFunction<LEDColor> function;
            InterpolationStyle(InterpolationFunction<Double> function) {
                this.function = LEDColor.transformInterpolator(function);
            }
            @Override
            public LEDColor interpolate(double t, LEDColor[] data) {return function.interpolate(t, data);}
        }

        public BasicGradient(InterpolationFunction<LEDColor> interpolationFunction, LEDColor... colors) {
            this.colors = colors;
            this.interpolationFunction = interpolationFunction;
        }

        @Override
        public LEDColor getColor(double x) {
            return interpolationFunction.interpolate(x, colors);
        }
    }

    public static final Gradient blackToWhite = (double x) -> new RGBColor(x,x,x);
    public static final Gradient rainbow = (double x) -> {
        final DoubleUnaryOperator sinusoid = (double X) -> Math.max(Math.cos(2 * Math.PI * X) * (2.0 / 3) + (1.0 / 3), 0);
        return new RGBColor(
            sinusoid.applyAsDouble(x - 0.0/3), 
            sinusoid.applyAsDouble(x - 1.0/3), 
            sinusoid.applyAsDouble(x - 2.0/3)
        );
    };
    public static final Gradient hueShift = (double x) -> new HSVColor(x,1,1);
}
