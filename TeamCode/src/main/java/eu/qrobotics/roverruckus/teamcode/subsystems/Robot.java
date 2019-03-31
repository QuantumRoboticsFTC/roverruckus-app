package eu.qrobotics.roverruckus.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;

@Config
public class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {
    public static final String TAG = "Robot";
    public static final String VUFORIA_KEY = "AZeyMpr/////AAABmbEShUcKiUwjoZ6YAwEmv09dz3UqbaI1dYslOuqZi1df8jYNLKBadOXeFjLRI/cJuDvGJC2VLghjm+sIy2YJf3nDHzg8jMZTTp1QPHPtXIIrEpOra6eimb69W+VCjZwW+lR1HyPVX56wJjgcThGEcdqC9j/rQoSKoKFCY+rZOOxG30rqJuYW3wKO97vlepFai4uTZ67Ipm7T9Hfrc+bUWZd+g2BqmweHvtDlixyab8TKsc8wTduOQBJ5Nxrh8ZmuCw/3SNZgIxOkacjzSJAT3dip2Q/VeKl1CG5SmxYy92GlwWWELm6tkCq9b+cS1GxvetnmsoRWUz0/oj/cuu0eRsrQmrYj3bV2Y9f7qecxgMSE";

    private ExpansionHubEx hub1;
    private ExpansionHubEx hub2;
    private RevBulkData revBulkData1;
    private RevBulkData revBulkData2;

    public MecanumDrive drive;
    public Intake intake;
    public Outtake outtake;
    public Climb climb;

    private List<Subsystem> subsystems;
    private List<Subsystem> subsystemsWithProblems;
    private List<CountDownLatch> cycleLatches;
    private ExecutorService subsystemUpdateExecutor;
    public FtcDashboard dashboard;
    public MovingStatistics top250, top100, top10;

    private boolean started;

    private static double getCurrentTime() {
        return System.nanoTime() / 1_000_000_000.0;
    }

    private Runnable subsystemUpdateRunnable = () -> {
        double startTime, temp;
        while (!Thread.currentThread().isInterrupted()) {
            try {
                startTime = getCurrentTime();
                revBulkData1 = hub1.getBulkInputData();
                revBulkData2 = hub2.getBulkInputData();
                for (Subsystem subsystem : subsystems) {
                    if (subsystem == null) continue;
                    try {
                        subsystem.update();
                        synchronized (subsystemsWithProblems) {
                            if (subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.remove(subsystem);
                            }
                        }
                    } catch (Throwable t) {
                        Log.w(TAG, "Subsystem update failed for " + subsystem.getClass().getSimpleName() + ": " + t.getMessage());
                        Log.w(TAG, t);
                        synchronized (subsystemsWithProblems) {
                            if (!subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.add(subsystem);
                            }
                        }
                    }
                    synchronized (cycleLatches) {
                        int i = 0;
                        while (i < cycleLatches.size()) {
                            CountDownLatch latch = cycleLatches.get(i);
                            latch.countDown();
                            if (latch.getCount() == 0) {
                                cycleLatches.remove(i);
                            } else {
                                i++;
                            }
                        }
                    }
                }
                temp = getCurrentTime() - startTime;
                top10.add(temp);
                top100.add(temp);
                top250.add(temp);
            } catch (Throwable t) {
                Log.wtf(TAG, t);
            }
        }
    };

    public Robot(OpMode opMode, boolean isAutonomous) {
        top10 = new MovingStatistics(10);
        top100 = new MovingStatistics(100);
        top250 = new MovingStatistics(250);
        dashboard = FtcDashboard.getInstance();
        RevExtensions2.init();

        hub1 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        hub2 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        revBulkData1 = hub1.getBulkInputData();
        revBulkData2 = hub2.getBulkInputData();

        subsystems = new ArrayList<>();
        try {
            drive = new MecanumDrive(opMode.hardwareMap, this, isAutonomous);
            subsystems.add(drive);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping MecanumDrive");
        }

        try {
            intake = new Intake(opMode.hardwareMap, this);
            subsystems.add(intake);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping Intake");
        }

        try {
            outtake = new Outtake(opMode.hardwareMap, this);
            subsystems.add(outtake);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping Outtake");
        }

        try {
            climb = new Climb(opMode.hardwareMap, this);
            subsystems.add(climb);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping Climb");
        }

        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");

        subsystemsWithProblems = new ArrayList<>();

        cycleLatches = new ArrayList<>();

    }

    public void start() {
        if (!started) {
            subsystemUpdateExecutor.submit(subsystemUpdateRunnable);
            started = true;
        }
    }

    public void stop() {
        if (started) {
            if (subsystemUpdateExecutor != null) {
                subsystemUpdateExecutor.shutdownNow();
                subsystemUpdateExecutor = null;
            }
        }
    }

    public ExpansionHubEx getHub1() {
        return hub1;
    }

    public ExpansionHubEx getHub2() {
        return hub2;
    }

    public RevBulkData getRevBulkDataHub1() {
        return revBulkData1;
    }

    public RevBulkData getRevBulkDataHub2() {
        return revBulkData2;
    }

    public void waitForNextCycle() {
        CountDownLatch latch = new CountDownLatch(1);
        synchronized (cycleLatches) {
            cycleLatches.add(latch);
        }
        try {
            latch.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void waitOneFullCycle() {
        CountDownLatch latch = new CountDownLatch(2);
        synchronized (cycleLatches) {
            cycleLatches.add(latch);
        }
        try {
            latch.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void sleep(double seconds) {
        try {
            Thread.sleep(Math.round(1000 * seconds));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        stop();
    }

    @Override
    public String getGlobalWarning() {
        List<String> warnings = new ArrayList<>();
        synchronized (subsystemsWithProblems) {
            for (Subsystem subsystem : subsystemsWithProblems) {
                warnings.add("Problem with " + subsystem.getClass().getSimpleName());
            }
        }
        return RobotLog.combineGlobalWarnings(warnings);
    }

    @Override
    public void suppressGlobalWarning(boolean suppress) {

    }

    @Override
    public void setGlobalWarning(String warning) {

    }

    @Override
    public void clearGlobalWarning() {
        synchronized (subsystemsWithProblems) {
            subsystemsWithProblems.clear();
        }
    }
}
