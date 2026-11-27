package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Config.*;

import java.util.List;

public class Limelight {
    private Limelight3A limelight;
    private LLResult currentResult;
    private LLResult lastGoodResult;
    private LLResult finalResult;

    private long lastGoodTime = 0;

    public boolean tVisible = false;
    public double tx;
    public double ty;
    public String[] debugString = new String[4];

    public Limelight(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT);
        limelight.pipelineSwitch(0); //set pipeline index
        limelight.setPollRateHz(100);
        start();
    }

    public void start(){limelight.start();}


    public void update(Alliance.Hive activeHive) {
        currentResult = limelight.getLatestResult();

        // Only use the latest camera frame.
        // Never feed an old AprilTag measurement into auto aim.
        if (currentResult != null && currentResult.isValid()) {
            finalResult = currentResult;
        } else {
            finalResult = null;
        }

        tVisible = getBestTVisible(activeHive);

        tx = tVisible ? getBestTX(activeHive) : 0;
        ty = tVisible ? finalResult.getTy() : 0;
    }

    public boolean isActiveHiveTag(int id, Alliance.Hive activeHive){ return activeHive.allTagIds.contains(id);}

    public boolean isMiddleTag(int id, Alliance.Hive activeHive){return activeHive.middleTagIds.contains(id);}

    public boolean getBestTVisible(Alliance.Hive activeHive){
        if (finalResult == null) { //safety checks in case finalResult is null
            return false;
        }
        List<LLResultTypes.FiducialResult> fiducials = finalResult.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {//return 0 if no tags
            return false;
        }
        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            if (isActiveHiveTag(fiducial.getFiducialId(), activeHive)) {
                return true;
            }
        }
        return false;
    }

    //it gets the best tx using the following algorithm:
    //always prioritize returning the tx positions of middle tags
    //maintain outer tags as backups in case no middle tags are seen
    //if AT LEAST two middle tags are seen it will average together the tx
    //otherwise if it sees two outer tags it will average together the tx

    public double getBestTX(Alliance.Hive activeHive){
        if (finalResult == null) {
            return 0;
        }

        List<LLResultTypes.FiducialResult> fiducials =
                finalResult.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            return 0;
        }

        double middleAverage = 0;
        int middleCount = 0;

        double backupAverage = 0;
        int backupCount = 0;

        debugString = new String[fiducials.size()]; //del
        int index = 0; //del

        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            int id = fiducial.getFiducialId();

            // Ignore tags belonging to other hives
            if (!isActiveHiveTag(id, activeHive)) {
                continue;
            }

            double tx = fiducial.getTargetXDegrees();

//            debugString = activeHive.name +
//                    " | ID: " + id +
//                    " | TX: " + tx +
//                    " | Middle: " + isMiddleTag(id, activeHive);
//            telemetry.addData("Limelight Debug", debugString);
            debugString[index] =
                    activeHive.name +
                    " | ID: " + id +
                    " | TX: " + tx +
                    " | Middle: " + isMiddleTag(id, activeHive);
            index++;

            if (isMiddleTag(id, activeHive)) {

                // Prioritize the inner two tags
                middleAverage += tx;
                middleCount++;

            } else {

                // Keep outer tags as backup
                backupAverage += tx;
                backupCount++;
            }
        }

        // Priority 1: Average all visible middle tags
        if (middleCount > 0) {
            return middleAverage / middleCount;
        }

        // Priority 2: Average all visible outer tags
        if (backupCount > 0) {
            return backupAverage / backupCount;
        }

        return 0;
    }


    public void stop(){limelight.stop();}
}
