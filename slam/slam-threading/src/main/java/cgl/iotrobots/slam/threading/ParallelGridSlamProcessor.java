package cgl.iotrobots.slam.threading;

import cgl.iotrobots.slam.core.gridfastsalm.SharedMemoryGridSlamProcessor;

import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.Semaphore;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class ParallelGridSlamProcessor extends SharedMemoryGridSlamProcessor {
    private int parallelism = 8;

    private ThreadPoolExecutor executor = new ThreadPoolExecutor(10, 10, 5, TimeUnit.SECONDS, new LinkedBlockingQueue<Runnable>());

    public void setParallelism(int parallelism) {
        this.parallelism = parallelism;
    }

    @Override
    public void scanMatch(double[] plainReading) {
        Semaphore semaphore = new Semaphore(particles.size());

        int noParticles = particles.size();

        for (int j = 0; j < noParticles; j++) {
            try {
                semaphore.acquire();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        for (int i = 0; i < parallelism; i++) {
            int particlesPerThread = noParticles / parallelism;
            if (particlesPerThread == 0) {
                particlesPerThread = 1;
            }

            int start = i * particlesPerThread;
            int end = i * particlesPerThread + particlesPerThread - 1;

            if (end >= noParticles) {
                end = noParticles - 1;
            } else if (i == parallelism - 1) {
                end = noParticles - 1;
            }

            if (start < noParticles) {
                executor.execute(new ScanMatchingWorker(particles, start, end, matcher, plainReading, minimumScore, semaphore));
            }
        }

        for (int i = 0; i < noParticles; i++) {
            try {
                semaphore.acquire();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void setup() {
    }
}
