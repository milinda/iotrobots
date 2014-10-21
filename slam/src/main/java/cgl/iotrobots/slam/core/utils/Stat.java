package cgl.iotrobots.slam.core.utils;

import cgl.iotrobots.slam.core.sensor.RangeReading;

import java.util.Random;

public class Stat {
    public static double sampleGaussian(double sigma, int S) {
      /*
        static gsl_rng * r = NULL;
        if(r==NULL) {
            gsl_rng_env_setup();
            r = gsl_rng_alloc (gsl_rng_default);
        }
        */
        Random random = new Random();
        if (S != 0) {
            //gsl_rng_set(r, S);
            random = new Random(S);
        }
        if (sigma == 0)
            return 0;
        //return gsl_ran_gaussian (r,sigma);
        return pf_ran_gaussian(sigma, random);
    }

    public static double pf_ran_gaussian(double sigma, Random random) {
        double x1, x2, w;
        double r;

        do {
            do {
                r = random.nextDouble();
            } while (r == 0.0);
            x1 = 2.0 * r - 1.0;
            do {
                r = random.nextDouble();
            } while (r == 0.0);
            x2 = 2.0 * random.nextDouble() - 1.0;
            w = x1 * x1 + x2 * x2;
        } while (w > 1.0 || w == 0.0);

        return (sigma * x2 * Math.sqrt(-2.0 * Math.log(w) / w));
    }
}
