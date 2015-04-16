mtype = {OFF, RED, GREEN, ORANGE, WALK, DONT_WALK};

bit whoIsGreenL = 0;
bit whoIsGreenT = 0;

mtype T0 = OFF;
mtype T1 = OFF;
mtype L0 = OFF;
mtype L1 = OFF;

mtype VEHICLE_SIGNALS = {OFF, RED, GREEN, ORANGE};
mtype PEDESTRIAN_SIGNALS = {OFF, WALK, DONT_WALK};
mtype EVENTS = {INIT, ADVANCE, PRE_STOP, STOP, ALL_STOP}

chan ch_toT[2] = [1] of { mtype };
chan ch_toL[2] = [1] of { mtype };


/* Client = Operator(of intersection controller) = Main */
/* A Kickstarter, where scheduling logic runs in its own thread */
proctype Operator {

}

proctype Scheduler {

}

proctype LinearLightSet (bit i) {

}

proctype TurnLightSet (bit i) {

}
