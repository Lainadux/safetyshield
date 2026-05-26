package Scenarios;

import Scenarios.Engine.HighwayAiClient;
import Scenarios.Engine.HighwayEngine;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;

public class GenerateInitialLogsRun {
    private static final int LOG_COUNT = 100;
    private static final int THREAD_COUNT = 4;
    private static final String LOG_DIR = StarkScenarioRunner.DEFAULT_LOG_DIR;

    public static void main(String[] args) throws Exception {
        int workerCount = Math.max(1, Math.min(THREAD_COUNT, LOG_COUNT));
        ExecutorService executor = Executors.newFixedThreadPool(workerCount);
        List<Future<?>> futures = new ArrayList<>();
        AtomicInteger completed = new AtomicInteger(0);

        try {
            for (int i = 0; i < LOG_COUNT; i++) {
                final int logIndex = i + 1;
                futures.add(executor.submit(() -> {
                    System.out.printf("Generating log %d/%d%n", logIndex, LOG_COUNT);
                    HighwayEngine realWorld = StarkScenarioRunner.createRandomWorld(StarkScenarioRunner.DEFAULT_DT);

                    StarkScenarioRunner.RunOptions options = new StarkScenarioRunner.RunOptions();
                    options.pauseAfterShieldDecision = false;
                    options.saveInitialState = true;
                    options.renderEachStep = false;
                    options.rethrowOnCrash = false;
                    options.printDiagnostics = false;
                    options.logDir = LOG_DIR;

                    StarkScenarioRunner.runShieldedScenario(realWorld, options);
                    int done = completed.incrementAndGet();
                    System.out.printf("Finished log %d/%d%n", done, LOG_COUNT);
                    return null;
                }));
            }

            for (Future<?> future : futures) {
                future.get();
            }
        } finally {
            executor.shutdownNow();
            HighwayAiClient.stopAll();
        }
    }
}
