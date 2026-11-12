package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.Config.Alliance;
import static org.firstinspires.ftc.teamcode.Config.currentAlliance;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controllers.AutoAimController;
import org.firstinspires.ftc.teamcode.controllers.IntakeTransferController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.PrematchSetup;

import java.util.ArrayList;

@Configurable
@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {

    public static final int START_FRONT = 0;
    public static final int START_REAR = 1;
    public static final int START_WALL = 2;
    public static final String[] START_NAMES = {"FRONT", "REAR", "WALL"};

    public static final int FLOWERS_NONE = 0;
    public static final int FLOWERS_AUDIENCE = 1;
    public static final int FLOWERS_WALL = 2;
    public static final int FLOWERS_BOTH = 3;
    public static final String[] FLOWER_NAMES = {"NONE", "AUDIENCE", "WALL", "BOTH"};

    public static final int PLAN_COUNT = 21;
    public static final int AUTO_PICK = 0;

    public static int startSide = PrematchSetup.loadInt("startSide", START_FRONT);
    public static boolean partnerShootsFirst = PrematchSetup.loadBoolean("partnerShootsFirst", false);
    public static double partnerShootWaitS = PrematchSetup.loadDouble("partnerShootWait", 5);
    public static int partnerFlowers = PrematchSetup.loadInt("partnerFlowerChoice", FLOWERS_NONE);
    public static boolean partnerTakesGarden = PrematchSetup.loadBoolean("partnerTakesGarden", false);
    public static int planPick = PrematchSetup.loadInt("planPick", AUTO_PICK);

    public static int PRELOAD_SHOTS = 7;
    public static int FLOWER_SHOTS = 3;
    public static int GARDEN_SHOTS = 3;
    public static long FLOWER_INTAKE_MS = 1500;
    public static long GARDEN_INTAKE_MS = 1000;
    public static double STOP_BY_S = 26;
    public static double SIDE_LANE_X = 138;
    public static boolean PARK_ENABLED = true;

    public static long AIM_MS = 700;
    public static long SHOT_OPEN_MS = 1400;
    public static long SHOT_GAP_MS = 400;
    public static int GATE_OPENS_PER_SHOOT = 1;
    public static boolean NECTAR_GATE_ONLY = false;
    public static double SHOT_FLYWHEEL_VELOCITY = 1126;
    public static boolean RAMP_ENABLED = true;
    public static double RAMP_START_MS = 100;
    public static double RAMP_DURATION_MS = 300;
    public static double RAMP_AMOUNT = 29;
    public static long SHOT_FINISH_MS = 750;
    public static double SETTLE_DISTANCE_IN = 1.5;
    public static long SETTLE_TIMEOUT_MS = 1500;

    public static double EST_SPEED_IN_PER_S = 35;
    public static double EST_SETTLE_S = 0.5;

    public static class Field {
        public final Alliance alliance;
        public final Alliance.Hive frontCell;
        public final Alliance.Hive rearCell;
        public final PoseFactory poses;
        public final Pose startFront, shootFront, startRear, shootRear, startWall;
        public final Pose audienceFlower, audienceFlowerLook, audienceFlowerShot;
        public final Pose wallFlower, wallFlowerLook, wallFlowerShot;
        public final Pose garden, gardenLook, garden2, garden2Look, gardenShot;
        public final Pose park, parkLook;
        public final double laneX;

        public Field(Alliance alliance) {
            this.alliance = alliance;
            frontCell = alliance.TOP;
            rearCell = alliance.BOTTOM;
            if (alliance == Alliance.BLUE) {
                poses = PoseFactory.degrees();
            } else {
                poses = PoseFactory.degrees().mirrorX(72).mirrorY(72);
            }

            startFront = poses.of(87.289, 140.3444, 89.69);
            shootFront = poses.of(87, 139, -90);
            startRear = poses.of(87.289, 3.6556, -89.69);
            shootRear = poses.of(87, 5, 90);
            startWall = poses.of(140.0219, 72.6705, 180);

            audienceFlower = poses.of(96.2249, 22.4006, -97.8131);
            audienceFlowerLook = poses.of(93.7, 4, 0);
            audienceFlowerShot = poses.of(84.237, 30.8644, 83.9652);

            wallFlower = poses.of(124.8986, 98.8559, 0.8421);
            wallFlowerLook = poses.of(134.7, 99, 0);
            wallFlowerShot = poses.of(86.3095, 133.7184, -90.3879);

            garden = poses.of(137.5695, 126.6587, 90);
            gardenLook = poses.of(138, 144, 0);
            garden2 = poses.of(137.5694, 139.1417, 90);
            garden2Look = poses.of(137.57, 150, 0);
            gardenShot = poses.of(86.3095, 133.7184, -90.3879);

            park = poses.of(133.6, 26, -86.4);
            parkLook = poses.of(135, 4, 0);

            laneX = poses.of(SIDE_LANE_X, 0, 0).x();
        }

        public Pose startPose(int side) {
            if (side == START_REAR) return startRear;
            if (side == START_WALL) return startWall;
            return startFront;
        }

        public Pose startShot(int side) {
            if (side == START_REAR) return shootRear;
            return shootFront;
        }

        public Alliance.Hive preloadHive(int side) {
            if (side == START_REAR) return rearCell;
            return frontCell;
        }

        public boolean crossesMiddle(Pose from, Pose to) {
            boolean topToBottom = from.y() > 72 && to.y() < 72;
            boolean bottomToTop = from.y() < 72 && to.y() > 72;
            return topToBottom || bottomToTop;
        }

        public Path parkPath(Pose from) {
            Path path;
            if (crossesMiddle(from, park)) {
                path = curve(from, new Pose(laneX, from.y(), 0), new Pose(laneX, park.y(), 0), park);
            } else {
                path = line(from, park);
            }
            return path.heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(parkLook)));
        }
    }

    public enum Kind {
        DRIVE, TARGET, WAIT_FOR_PARTNER, SHOOT, PAUSE, FLOWER_DOWN, FLOWER_UP,
        USES_WALL_FLOWER, USES_AUDIENCE_FLOWER, USES_GARDEN
    }

    public static class Move {
        public Kind kind;
        public Pose from;
        public Pose to;
        public Pose control1;
        public Pose control2;
        public Pose lookAt;
        public boolean backwards;
        public Alliance.Hive hive;
        public int shots;
        public long ms;
    }

    public static class PlanSteps {
        public final Field field;
        public final Pose start;
        public final ArrayList<Move> moves = new ArrayList<>();
        private Pose here;

        public PlanSteps(Field field, Pose start) {
            this.field = field;
            this.start = start;
            this.here = start;
        }

        private void addSimple(Kind kind) {
            Move move = new Move();
            move.kind = kind;
            moves.add(move);
        }

        private void addDrive(Pose to, Pose lookAt, boolean backwards, Pose control1, Pose control2) {
            Move move = new Move();
            move.kind = Kind.DRIVE;
            move.from = here;
            move.to = to;
            move.lookAt = lookAt;
            move.backwards = backwards;
            move.control1 = control1;
            move.control2 = control2;
            moves.add(move);
            here = to;
        }

        public void driveAim(Pose to, Alliance.Hive hive) {
            addDrive(to, hive.pose, true, null, null);
        }

        public void driveAim(Pose to, Alliance.Hive hive, Pose control1) {
            addDrive(to, hive.pose, true, control1, null);
        }

        public void driveAim(Pose to, Alliance.Hive hive, Pose control1, Pose control2) {
            addDrive(to, hive.pose, true, control1, control2);
        }

        public void driveFace(Pose to, Pose lookAt) {
            addDrive(to, lookAt, false, null, null);
        }

        public void driveFace(Pose to, Pose lookAt, Pose control1) {
            addDrive(to, lookAt, false, control1, null);
        }

        public void driveStraight(Pose to) {
            addDrive(to, null, false, null, null);
        }

        public void routeAim(Pose to, Alliance.Hive hive) {
            if (field.crossesMiddle(here, to)) {
                addDrive(to, hive.pose, true, new Pose(field.laneX, here.y(), 0), new Pose(field.laneX, to.y(), 0));
            } else {
                addDrive(to, hive.pose, true, null, null);
            }
        }

        public void routeFace(Pose to, Pose lookAt) {
            if (field.crossesMiddle(here, to)) {
                addDrive(to, lookAt, false, new Pose(field.laneX, here.y(), 0), new Pose(field.laneX, to.y(), 0));
            } else {
                addDrive(to, lookAt, false, null, null);
            }
        }

        public void target(Alliance.Hive hive) {
            Move move = new Move();
            move.kind = Kind.TARGET;
            move.hive = hive;
            moves.add(move);
        }

        public void shoot(int shots) {
            Move move = new Move();
            move.kind = Kind.SHOOT;
            move.shots = shots;
            moves.add(move);
        }

        public void pause(long ms) {
            Move move = new Move();
            move.kind = Kind.PAUSE;
            move.ms = ms;
            moves.add(move);
        }

        public void waitForPartner() {
            addSimple(Kind.WAIT_FOR_PARTNER);
        }

        public void flowerDown() {
            addSimple(Kind.FLOWER_DOWN);
        }

        public void flowerUp() {
            addSimple(Kind.FLOWER_UP);
        }

        public void usesWallFlower() {
            addSimple(Kind.USES_WALL_FLOWER);
        }

        public void usesAudienceFlower() {
            addSimple(Kind.USES_AUDIENCE_FLOWER);
        }

        public void usesGarden() {
            addSimple(Kind.USES_GARDEN);
        }

        public int shots() {
            int total = 0;
            for (Move move : moves) {
                if (move.kind == Kind.SHOOT) total = total + move.shots;
            }
            return total;
        }

        public boolean usesAnything() {
            for (Move move : moves) {
                if (move.kind == Kind.USES_WALL_FLOWER) return true;
                if (move.kind == Kind.USES_AUDIENCE_FLOWER) return true;
                if (move.kind == Kind.USES_GARDEN) return true;
            }
            return false;
        }

        public boolean blockedByPartner() {
            for (Move move : moves) {
                if (move.kind == Kind.USES_WALL_FLOWER && partnerTakesWallFlower()) return true;
                if (move.kind == Kind.USES_AUDIENCE_FLOWER && partnerTakesAudienceFlower()) return true;
                if (move.kind == Kind.USES_GARDEN && partnerTakesGarden) return true;
            }
            return false;
        }

        public double seconds() {
            double time = 0;
            for (Move move : moves) {
                if (move.kind == Kind.DRIVE) time = time + driveSeconds(move);
                if (move.kind == Kind.WAIT_FOR_PARTNER) time = Math.max(time, partnerWaitS());
                if (move.kind == Kind.SHOOT) time = time + shootSeconds();
                if (move.kind == Kind.PAUSE) time = time + move.ms / 1000.0;
            }
            return time;
        }

        public double secondsUntilFirstShot() {
            double time = 0;
            for (Move move : moves) {
                if (move.kind == Kind.SHOOT) return time + shootSeconds();
                if (move.kind == Kind.DRIVE) time = time + driveSeconds(move);
                if (move.kind == Kind.WAIT_FOR_PARTNER) time = Math.max(time, partnerWaitS());
            }
            return time;
        }

        public String describe() {
            String text = "";
            for (Move move : moves) {
                String part = "";
                if (move.kind == Kind.SHOOT) part = "shoot " + move.shots;
                if (move.kind == Kind.USES_GARDEN) part = "garden";
                if (move.kind == Kind.USES_WALL_FLOWER) part = "wall flower";
                if (move.kind == Kind.USES_AUDIENCE_FLOWER) part = "audience flower";
                if (part.isEmpty()) continue;
                if (!text.isEmpty()) text = text + " > ";
                text = text + part;
            }
            if (text.isEmpty()) return "stay";
            return text;
        }
    }

    public static int planSide(int plan) {
        if (plan == 2) return START_WALL;
        if (plan == 4 || plan == 6) return START_REAR;
        if (plan >= 14 && plan <= 20) return START_REAR;
        return START_FRONT;
    }

    public static boolean planFits(int plan, int side) {
        if (plan == 21) return side != START_WALL;
        return planSide(plan) == side;
    }

    public static PlanSteps makePlan(int plan, Field f, int side) {
        if (plan == 1) return plan1(f);
        if (plan == 2) return plan2(f);
        if (plan == 3) return plan3(f);
        if (plan == 4) return plan4(f);
        if (plan == 5) return plan5(f);
        if (plan == 6) return plan6(f);
        if (plan == 21) return new PlanSteps(f, f.startPose(side));
        return comboPlan(plan, f);
    }

    private static PlanSteps plan1(Field f) {
        PoseFactory p = f.poses;
        PlanSteps s = new PlanSteps(f, f.startFront);
        s.target(f.frontCell);
        s.driveAim(f.shootFront, f.frontCell);
        s.waitForPartner();
        s.shoot(PRELOAD_SHOTS);
        s.usesGarden();
        s.driveFace(p.of(136.1938, 126.6587, 84.0537), p.of(138, 144, 0), p.of(131.9595, 88.2177, 0));
        s.driveFace(p.of(136.1937, 139.1417, 90.0196), p.of(136.19, 150, 0));
        s.pause(2000);
        s.target(f.rearCell);
        s.driveAim(p.of(84.435, 31.0886, 84.3466), f.rearCell, p.of(90.8954, 112.0252, 0), p.of(140.0766, 21.794, 0));
        s.shoot(1);
        s.pause(2000);
        s.usesAudienceFlower();
        s.flowerDown();
        s.driveFace(f.audienceFlower, f.audienceFlowerLook);
        s.pause(FLOWER_INTAKE_MS);
        s.flowerUp();
        s.target(f.rearCell);
        s.driveAim(f.audienceFlowerShot, f.rearCell);
        s.shoot(1);
        s.pause(2000);
        s.usesWallFlower();
        s.flowerDown();
        s.driveFace(f.wallFlower, f.wallFlowerLook, p.of(114.0538, 25.2474, 0));
        s.pause(FLOWER_INTAKE_MS);
        s.flowerUp();
        return s;
    }

    private static PlanSteps plan2(Field f) {
        PoseFactory p = f.poses;
        PlanSteps s = new PlanSteps(f, f.startWall);
        s.driveStraight(p.of(133.4425, 72.7376, 179.4157));
        return s;
    }

    private static PlanSteps plan3(Field f) {
        PoseFactory p = f.poses;
        PlanSteps s = new PlanSteps(f, p.of(87.289, 140.3444, -90));
        s.target(f.frontCell);
        s.driveAim(p.of(87.1548, 133.3804, -91.4576), f.frontCell);
        s.waitForPartner();
        s.shoot(4);
        s.usesGarden();
        s.driveFace(p.of(139.5348, 126.8552, 95.1153), p.of(138, 144, 0), p.of(131.9595, 88.2177, 0));
        s.driveFace(p.of(139.5347, 141.5, 93.5996), p.of(139, 150, 0));
        s.pause(2000);
        s.target(f.rearCell);
        s.driveAim(p.of(84.435, 31.0886, -95.6534), f.rearCell, p.of(90.8954, 112.0252, 0), p.of(140.0766, 21.794, 0));
        s.shoot(4);
        s.usesAudienceFlower();
        s.flowerDown();
        s.driveFace(p.of(95.0457, 30.4583, -92.9117), f.audienceFlowerLook);
        s.pause(FLOWER_INTAKE_MS);
        s.flowerUp();
        s.target(f.frontCell);
        s.driveAim(p.of(86.5064, 133.4041, -90.639), f.frontCell, p.of(129.5497, 83.5041, 0));
        s.shoot(4);
        return s;
    }

    private static PlanSteps plan4(Field f) {
        PoseFactory p = f.poses;
        PlanSteps s = new PlanSteps(f, p.of(85.3292, 5.1915, 90));
        s.target(f.rearCell);
        s.driveAim(p.of(85.087, 11.8986, 88.9126), f.rearCell);
        s.waitForPartner();
        s.shoot(4);
        s.driveFace(p.of(132, 28, 83.3), p.of(140, 96, 0));
        if (f.alliance == Alliance.RED) {
            s.pause(2000);
        } else {
            s.pause(1750);
        }
        s.usesWallFlower();
        s.flowerDown();
        s.driveFace(p.of(115.4, 96.5859, -4.9781), p.of(140, 96, 0));
        s.pause(FLOWER_INTAKE_MS);
        s.flowerUp();
        s.target(f.frontCell);
        s.driveAim(f.wallFlowerShot, f.frontCell);
        s.shoot(4);
        s.driveFace(p.of(128, 134, -86.9), p.of(135, 4, 0));
        return s;
    }

    private static PlanSteps plan5(Field f) {
        PlanSteps s = new PlanSteps(f, f.startFront);
        s.target(f.frontCell);
        s.driveAim(f.shootFront, f.frontCell);
        s.waitForPartner();
        s.shoot(PRELOAD_SHOTS);
        return s;
    }

    private static PlanSteps plan6(Field f) {
        PlanSteps s = new PlanSteps(f, f.startRear);
        s.target(f.rearCell);
        s.driveAim(f.shootRear, f.rearCell);
        s.waitForPartner();
        s.shoot(PRELOAD_SHOTS);
        return s;
    }

    private static PlanSteps comboPlan(int plan, Field f) {
        PlanSteps s = preload(f, planSide(plan));
        if (plan == 7) { addWallFlower(s); addAudienceFlower(s); }
        if (plan == 8) { addGarden(s); addWallFlower(s); addAudienceFlower(s); }
        if (plan == 9) { addGarden(s); addWallFlower(s); }
        if (plan == 10) { addGarden(s); addAudienceFlower(s); }
        if (plan == 11) { addWallFlower(s); }
        if (plan == 12) { addAudienceFlower(s); }
        if (plan == 13) { addGarden(s); }
        if (plan == 14) { addAudienceFlower(s); addWallFlower(s); }
        if (plan == 15) { addAudienceFlower(s); addWallFlower(s); addGarden(s); }
        if (plan == 16) { addAudienceFlower(s); addGarden(s); }
        if (plan == 17) { addWallFlower(s); addGarden(s); }
        if (plan == 18) { addAudienceFlower(s); }
        if (plan == 19) { addWallFlower(s); }
        if (plan == 20) { addGarden(s); }
        return s;
    }

    private static PlanSteps preload(Field f, int side) {
        Alliance.Hive hive = f.preloadHive(side);
        PlanSteps s = new PlanSteps(f, f.startPose(side));
        s.target(hive);
        s.routeAim(f.startShot(side), hive);
        s.waitForPartner();
        s.shoot(PRELOAD_SHOTS);
        return s;
    }

    private static void addGarden(PlanSteps s) {
        Field f = s.field;
        s.usesGarden();
        s.routeFace(f.garden, f.gardenLook);
        s.routeFace(f.garden2, f.garden2Look);
        s.pause(GARDEN_INTAKE_MS);
        s.target(f.frontCell);
        s.routeAim(f.gardenShot, f.frontCell);
        s.shoot(GARDEN_SHOTS);
    }

    private static void addWallFlower(PlanSteps s) {
        Field f = s.field;
        s.usesWallFlower();
        s.flowerDown();
        s.routeFace(f.wallFlower, f.wallFlowerLook);
        s.pause(FLOWER_INTAKE_MS);
        s.flowerUp();
        s.target(f.frontCell);
        s.routeAim(f.wallFlowerShot, f.frontCell);
        s.shoot(FLOWER_SHOTS);
    }

    private static void addAudienceFlower(PlanSteps s) {
        Field f = s.field;
        s.usesAudienceFlower();
        s.flowerDown();
        s.routeFace(f.audienceFlower, f.audienceFlowerLook);
        s.pause(FLOWER_INTAKE_MS);
        s.flowerUp();
        s.target(f.rearCell);
        s.routeAim(f.audienceFlowerShot, f.rearCell);
        s.shoot(FLOWER_SHOTS);
    }

    private static double distance(Pose a, Pose b) {
        return Math.hypot(b.x() - a.x(), b.y() - a.y());
    }

    private static double driveSeconds(Move move) {
        double length;
        if (move.control1 == null) {
            length = distance(move.from, move.to);
        } else if (move.control2 == null) {
            length = distance(move.from, move.control1) + distance(move.control1, move.to);
        } else {
            length = distance(move.from, move.control1) + distance(move.control1, move.control2) + distance(move.control2, move.to);
        }
        return length / EST_SPEED_IN_PER_S + EST_SETTLE_S;
    }

    private static double shootSeconds() {
        double gateOpens = Math.max(1, GATE_OPENS_PER_SHOOT);
        return (AIM_MS + gateOpens * (SHOT_OPEN_MS + SHOT_GAP_MS) + SHOT_FINISH_MS) / 1000.0;
    }

    public static double partnerWaitS() {
        if (partnerShootsFirst) return partnerShootWaitS;
        return 0;
    }

    public static boolean partnerTakesWallFlower() {
        return partnerFlowers == FLOWERS_WALL || partnerFlowers == FLOWERS_BOTH;
    }

    public static boolean partnerTakesAudienceFlower() {
        return partnerFlowers == FLOWERS_AUDIENCE || partnerFlowers == FLOWERS_BOTH;
    }

    private static PlanSteps bluePlan(int plan) {
        return makePlan(plan, new Field(Alliance.BLUE), startSide);
    }

    private static boolean fitsInTime(PlanSteps s) {
        if (!s.usesAnything() && s.shots() > 0) {
            return s.secondsUntilFirstShot() <= STOP_BY_S;
        }
        return s.seconds() <= STOP_BY_S;
    }

    public static int pickedPlan() {
        if (planPick < 1 || planPick > PLAN_COUNT) return AUTO_PICK;
        if (!planFits(planPick, startSide)) return AUTO_PICK;
        return planPick;
    }

    public static int bestPlan() {
        int best = 21;
        if (startSide == START_WALL) best = 2;
        double bestTime = 0;
        int bestShots = 0;
        for (int plan = 1; plan <= PLAN_COUNT; plan++) {
            if (!planFits(plan, startSide)) continue;
            PlanSteps s = bluePlan(plan);
            if (s.blockedByPartner()) continue;
            if (!fitsInTime(s)) continue;
            double time = s.seconds();
            int shots = s.shots();
            boolean moreShots = shots > bestShots;
            boolean sameShotsButFaster = shots == bestShots && time < bestTime;
            if (moreShots || sameShotsButFaster) {
                best = plan;
                bestTime = time;
                bestShots = shots;
            }
        }
        return best;
    }

    public static int chosenPlan() {
        if (pickedPlan() != AUTO_PICK) return pickedPlan();
        return bestPlan();
    }

    public static ArrayList<Integer> planChoices() {
        ArrayList<Integer> choices = new ArrayList<>();
        choices.add(AUTO_PICK);
        for (int plan = 1; plan <= PLAN_COUNT; plan++) {
            if (planFits(plan, startSide)) choices.add(plan);
        }
        return choices;
    }

    public static String planName(int plan) {
        if (plan == AUTO_PICK) return "AUTO";
        return "Plan " + plan;
    }

    public static String startText() {
        Pose s = bluePlan(chosenPlan()).start;
        return String.format("x %.1f  y %.1f  heading %.0f (blue side)", s.x(), s.y(), Math.toDegrees(s.heading()));
    }

    public static String planSummary() {
        int plan = chosenPlan();
        PlanSteps s = bluePlan(plan);
        String text = planName(plan);
        if (pickedPlan() == AUTO_PICK) {
            text = text + " (auto)";
        } else {
            text = text + " (picked)";
        }
        text = text + String.format(" ~%.1fs, %d shots: ", s.seconds(), s.shots());
        if (partnerShootsFirst) {
            text = text + "hold fire until " + partnerShootWaitS + "s > ";
        }
        text = text + s.describe();
        if (PARK_ENABLED) text = text + " > park";
        return text;
    }

    private Follower follower;
    private IntakeTransferController intake;
    private OuttakeController outtake;
    private AutoAimController autoAim;

    private Field field;
    private int plan;
    private PlanSteps planSteps;
    private Alliance.Hive targetHive = null;
    private boolean aiming = false;
    private boolean gateOpen = false;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();

    private int stepNumber = 0;
    private boolean stepStarted = false;
    private boolean autoDone = false;
    private boolean parking = false;
    private int shootPart = 0;
    private int gateOpensDone = 0;

    private void openGate() {
        if (NECTAR_GATE_ONLY) {
            outtake.fireNectar();
        } else {
            outtake.fire();
        }
        gateOpen = true;
        gateTimer.reset();
    }

    private void closeGate() {
        outtake.close();
        gateOpen = false;
    }

    private void stopShooting() {
        aiming = false;
        targetHive = null;
    }

    private double shotVelocity() {
        if (!RAMP_ENABLED || !gateOpen) return SHOT_FLYWHEEL_VELOCITY;
        double msIntoRamp = gateTimer.milliseconds() - RAMP_START_MS;
        if (msIntoRamp <= 0) return SHOT_FLYWHEEL_VELOCITY;
        double fraction = 1;
        if (RAMP_DURATION_MS > 0) {
            fraction = Math.min(1, msIntoRamp / RAMP_DURATION_MS);
        }
        return SHOT_FLYWHEEL_VELOCITY + RAMP_AMOUNT * fraction;
    }

    private Path buildPath(Move move) {
        Path path;
        if (move.control1 == null) {
            path = line(move.from, move.to);
        } else if (move.control2 == null) {
            path = curve(move.from, move.control1, move.to);
        } else {
            path = curve(move.from, move.control1, move.control2, move.to);
        }

        if (move.lookAt == null) {
            return path.tangent();
        }
        if (move.backwards) {
            return path.heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move.lookAt).reverse()));
        }
        return path.heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move.lookAt)));
    }

    private void startStep(Move move) {
        stepTimer.reset();
        if (move.kind == Kind.DRIVE) follower.follow(buildPath(move));
        if (move.kind == Kind.TARGET) targetHive = move.hive;
        if (move.kind == Kind.FLOWER_DOWN) intake.flowerIntakeDown();
        if (move.kind == Kind.FLOWER_UP) intake.flowerIntakeUp();
        if (move.kind == Kind.SHOOT) {
            shootPart = 0;
            gateOpensDone = 0;
        }
    }

    private boolean isStepDone(Move move) {
        if (move.kind == Kind.DRIVE) return follower.atParametricEnd();
        if (move.kind == Kind.WAIT_FOR_PARTNER) return autoTimer.seconds() >= partnerWaitS();
        if (move.kind == Kind.PAUSE) return stepTimer.milliseconds() >= move.ms;
        if (move.kind == Kind.SHOOT) return updateShooting();
        return true;
    }

    private boolean updateShooting() {
        if (shootPart == 0) {
            boolean settled = follower.distanceToEndpoint() < SETTLE_DISTANCE_IN;
            boolean tooLong = stepTimer.milliseconds() > SETTLE_TIMEOUT_MS;
            if (settled || tooLong) {
                aiming = true;
                stepTimer.reset();
                shootPart = 1;
            }
        }
        if (shootPart == 1 && stepTimer.milliseconds() >= AIM_MS) {
            openGate();
            stepTimer.reset();
            shootPart = 2;
        }
        if (shootPart == 2 && stepTimer.milliseconds() >= SHOT_OPEN_MS) {
            closeGate();
            stepTimer.reset();
            shootPart = 3;
        }
        if (shootPart == 3 && stepTimer.milliseconds() >= SHOT_GAP_MS) {
            gateOpensDone = gateOpensDone + 1;
            stepTimer.reset();
            if (gateOpensDone < Math.max(1, GATE_OPENS_PER_SHOOT)) {
                openGate();
                shootPart = 2;
            } else {
                shootPart = 4;
            }
        }
        if (shootPart == 4 && stepTimer.milliseconds() >= SHOT_FINISH_MS) {
            stopShooting();
            return true;
        }
        return false;
    }

    private void finishAuto() {
        stopShooting();
        closeGate();
        intake.flowerIntakeUp();
        Pose robot = follower.pose();
        if (PARK_ENABLED && distance(robot, field.park) > SETTLE_DISTANCE_IN) {
            follower.follow(field.parkPath(robot));
            parking = true;
            return;
        }
        autoDone = true;
    }

    private void runSteps() {
        if (autoDone) return;
        if (parking) {
            if (follower.atParametricEnd()) {
                parking = false;
                autoDone = true;
            }
            return;
        }
        if (autoTimer.seconds() > STOP_BY_S) {
            finishAuto();
            return;
        }
        while (stepNumber < planSteps.moves.size()) {
            Move move = planSteps.moves.get(stepNumber);
            if (!stepStarted) {
                startStep(move);
                stepStarted = true;
            }
            if (!isStepDone(move)) return;
            stepNumber = stepNumber + 1;
            stepStarted = false;
        }
        finishAuto();
    }

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        intake = new IntakeTransferController(hardwareMap);
        outtake = new OuttakeController(hardwareMap);
        autoAim = new AutoAimController(hardwareMap);
        if (currentAlliance == null) {
            field = new Field(Alliance.BLUE);
        } else {
            field = new Field(currentAlliance);
        }
        plan = chosenPlan();
        planSteps = makePlan(plan, field, startSide);
        follower.setPose(planSteps.start);
        follower.update();

        while (opModeInInit()) {
            telemetry.addLine("=== AUTO ===");
            telemetry.addData("Alliance", field.alliance.name());
            telemetry.addData("We start", START_NAMES[startSide]);
            if (partnerShootsFirst) {
                telemetry.addData("Shoots first", "PARTNER");
                telemetry.addData("Partner shoot wait (s)", partnerShootWaitS);
            } else {
                telemetry.addData("Shoots first", "US");
            }
            telemetry.addData("Partner takes flower", FLOWER_NAMES[partnerFlowers]);
            if (partnerTakesGarden) {
                telemetry.addData("Partner takes garden", "YES");
            } else {
                telemetry.addData("Partner takes garden", "NO");
            }
            telemetry.addData("Plan pick", planName(planPick));
            telemetry.addLine("");
            telemetry.addLine("PLAN: " + planSummary());
            telemetry.addLine("Place robot: " + startText());
            telemetry.addLine("");
            telemetry.addLine("Change answers in Prematch Setup");
            telemetry.update();
            idle();
        }

        if (!opModeIsActive()) {
            OpModeStorage.autonomousEndPose = follower.pose();
            return;
        }

        autoTimer.reset();
        intake.toggleForward();
        outtake.close();

        while (opModeIsActive()) {
            follower.update();
            runSteps();

            Pose robot = follower.pose();
            if (targetHive != null) {
                autoAim.update(targetHive, robot);
            }
            outtake.flywheel.setVelocity(shotVelocity());
            outtake.flywheel.update();
            intake.update();

            if (aiming) {
                ManualDrive.driveOrHold(follower, new DrivePowers(0, 0, autoAim.rotation));
            }

            telemetry.addData("plan", planName(plan));
            telemetry.addData("auto time", autoTimer.seconds());
            telemetry.addData("parking", parking);
            if (targetHive == null) {
                telemetry.addData("target", "none");
            } else {
                telemetry.addData("target", targetHive.name);
            }
            telemetry.addData("x", robot.x());
            telemetry.addData("y", robot.y());
            telemetry.addData("heading", Math.toDegrees(robot.heading()));
            telemetry.update();
        }

        OpModeStorage.autonomousEndPose = follower.pose();
    }
}
