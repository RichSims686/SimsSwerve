package frc.robot.util.led;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.InterpolationFunction;
import frc.robot.util.led.functions.Gradient;

public interface LEDColor extends Gradient {
   @Override
   public default LEDColor getColor(double x) {return this;}

   public static final RGBColor Red =     new RGBColor(255,   0,   0);
   public static final RGBColor Yellow =  new RGBColor(255, 255,   0);
   public static final RGBColor Green =   new RGBColor(  0, 255,   0);
   public static final RGBColor Cyan =    new RGBColor(  0, 255, 255);
   public static final RGBColor Blue =    new RGBColor(  0,   0, 255);
   public static final RGBColor Purple =  new RGBColor(255,   0, 255);
   public static final RGBColor Black =   new RGBColor(  0,   0,   0);
   public static final RGBColor White =   new RGBColor(255, 255, 255);

   public RGBColor toRGB();
   public default double getWhiteness() {return 0;}

   public static class RawLEDColor {
      public final int r;
      public final int g;
      public final int b;
      public final int w;
      public RawLEDColor(int r, int g, int b, int w) {
         this.r = MathUtil.clamp(r, 0, 255);
         this.g = MathUtil.clamp(g, 0, 255);
         this.b = MathUtil.clamp(b, 0, 255);
         this.w = MathUtil.clamp(w, 0, 255);
      }
      @Override
      public boolean equals(Object obj) {
         boolean r = true;
         if(obj instanceof RawLEDColor) {
            var otherColor = (RawLEDColor)obj;
            r = r && (this.r == otherColor.r);
            r = r && (this.g == otherColor.g);
            r = r && (this.b == otherColor.b);
            r = r && (this.w == otherColor.w);
         } else {
            r = false;
         }
         return r;
      }
   }
   public default RawLEDColor getLEDColor() {
      var color = toRGB();
      return new RawLEDColor(
         (int)Math.round(color.getRed()*255), 
         (int)Math.round(color.getGreen()*255), 
         (int)Math.round(color.getBlue()*255),
         (int)Math.round(color.getWhiteness()*255)
      );
   }
   
   public static InterpolationFunction<LEDColor> transformInterpolator(InterpolationFunction<Double> interpolator) {
      return (double t, LEDColor[] data) -> {
         Double[] r = new Double[data.length];
         Double[] g = new Double[data.length];
         Double[] b = new Double[data.length];
         for(int i = 0; i < data.length; i++) {
            var color = data[i].toRGB();
            r[i] = color.getRed();
            g[i] = color.getGreen();
            b[i] = color.getBlue();
         }
         return new RGBColor(
            interpolator.interpolate(t, r), 
            interpolator.interpolate(t, g), 
            interpolator.interpolate(t, b)
         );
      };
   }

   public static class RGBColor implements LEDColor {
      private double    red;
      public RGBColor   setRed(double red)      {this.red = MathUtil.clamp(red , 0, 255); return this;}
      public RGBColor   setR(int r)             {return setRed(r/255.0);}
      public double     getRed()                {return red;}

      private double    green;
      public RGBColor   setGreen(double green)  {this.green = MathUtil.clamp(green, 0, 255); return this;}
      public RGBColor   setG(int g)             {return setGreen(g/255.0);}
      public double     getGreen()              {return green;}
      
      private double    blue;
      public RGBColor   setBlue(double blue)    {this.blue = MathUtil.clamp(blue, 0, 255); return this;}
      public RGBColor   setB(int b)             {return setBlue(b/255.0);}
      public double     getBlue()               {return blue;}

      public RGBColor() {}
      /**
       * 
       * @param red - Red value | Range [0-1]
       * @param green - Green value | Range [0-1]
       * @param blue - Blue value | Range [0-1]
       */
      public RGBColor(double red, double green, double blue) {
         setRed(red);
         setGreen(green);
         setBlue(blue);
      }
      /**
       * 
       * @param r - Red value | Range [0-255]
       * @param g - Green value | Range [0-255]
       * @param b - Blue value | Range [0-255]
       */
      public RGBColor(int r, int g, int b) {
         setR(r);
         setG(g);
         setB(b);
      }

      @Override
      public RGBColor toRGB() {
         return this;
      }
   }

   public static class HSVColor implements LEDColor {
      private double    hue;
      public HSVColor   setHue(double hue)   {this.hue = MathUtil.clamp(hue, 0, 255); return this;}
      public HSVColor   setH(int h)          {return setHue(h/255.0);}
      public double     getHue()             {return hue;}

      private double    saturation;
      public HSVColor   setSaturation(double saturation) {this.saturation = MathUtil.clamp(saturation, 0, 255); return this;}
      public HSVColor   setS(int s)                      {return setSaturation(s/255.0);}
      public double     getSaturation()                  {return saturation;}
      
      private double    value;
      public HSVColor   setValue(double value)  {this.value = MathUtil.clamp(value, 0, 255); return this;}
      public HSVColor   setV(int v)             {return setValue(v/255.0);}
      public double     getValue()              {return value;}

      public HSVColor() {}
      /**
       * 
       * @param hue - Hue value | Range [0-1]
       * @param saturation - Saturation value | Range [0-1]
       * @param value - Value value | Range [0-1]
       */
      public HSVColor(double hue, double saturation, double value) {
         setHue(hue);
         setSaturation(saturation);
         setValue(value);
      }
      /**
       * 
       * @param h - Hue value | Range [0-255]
       * @param s - Saturation value | Range [0-255]
       * @param v - Value value | Range [0-255]
       */
      public HSVColor(int h, int s, int v) {
         setH(h);
         setS(s);
         setV(v);
      }

      @Override
      public RGBColor toRGB() {
         // Follows the formula provided by Wikipedia
         double chroma = value*saturation;
         double hPrime = hue * 6;
         double x = chroma * (1 - Math.abs(hPrime % 2 - 1));
         double m = value - chroma;
         switch((int)hPrime) {
            case 0:  return new RGBColor(chroma + m, x + m, m);
            case 1:  return new RGBColor(x + m, chroma + m, m);
            case 2:  return new RGBColor(m, chroma + m, x + m);
            case 3:  return new RGBColor(m, x + m, chroma + m);
            case 4:  return new RGBColor(x + m, m, chroma + m);
            case 5:  return new RGBColor(chroma + m, m, x + m);
            default: return new RGBColor();
         }
      }
   }
}
