package org.firstinspires.ftc.teamcode.util;
import org.firstinspires.ftc.teamcode.Profile;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad.*;
public class Drivers {
    Drivers drivers;

    public final static Profile AHMED = new Profile("Ahmed");
//    static{Ahmed.REMOVE_DRIFT = Button.B;}
    public final static Profile SPARSH = new Profile("Sparsh");
//    static{Sparsh.REMOVE_DRIFT = Button.B;}
    public static final Profile[] DRIVERS = {
            AHMED,
            SPARSH,
    };
    public static Profile activeDriver = DRIVERS[0]; // defaults to using top most driver as active!
}
