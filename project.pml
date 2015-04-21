mtype = {OFF, RED, GREEN, ORANGE, WALK, DONT_WALK};
mtype = {INIT, ADVANCE, PRE_STOP, STOP, ALL_STOP}

chan ch_toT[2] = [1] of { mtype };
chan ch_toL[2] = [1] of { mtype };

/* Current state of lights. */
mtype state_L[2];
mtype state_T[2];

/* Pedestrian Light. */
mtype state_P[2];

/* Mutex for light sets. Used for synchronization. */
bit mutex_L[2];
bit mutex_T[2];

/* Client = Operator(of intersection controller) = Main */
/* A Kickstarter, where scheduling logic runs in its own thread */
proctype Operator() {
  mutex_L[0] = 0;
  mutex_L[1] = 0;
  mutex_T[0] = 0;
  mutex_T[1] = 0;
  
  state_L[0] = OFF;
  state_L[1] = OFF;
  state_T[0] = OFF;
  state_T[1] = OFF;
  
  /*part of initialization*/
  
  //turnOnStopLights();
  mutex_L[0] == 0; /*wait for lock*/
  mutex_L[0] = 1; /*lock*/
  d_step
  {
    ch_toL[0]!INIT;
    mutex_L[0] = 0; /*unlock*/
  }
  mutex_L[1] == 0; /*wait for lock*/
  mutex_L[1] = 1; /*lock*/
  d_step
  {
    ch_toL[1]!INIT;
    mutex_L[1] = 0; /*unlock*/
  }
  //turnOnTurnLights();
  mutex_T[0] == 0; /*wait for lock*/
  mutex_T[0] = 1; /*lock*/
  d_step
  {
    ch_toT[0]!INIT;
    mutex_T[0] = 0; /*unlock*/
  }
  mutex_T[1] == 0; /*wait for lock*/
  mutex_T[1] = 1; /*lock*/
  d_step
  {
    ch_toT[1]!INIT;
    mutex_T[1] = 0; /*unlock*/
  }
  run Scheduler();
  
}

proctype Scheduler() {
  /*advanceStopLights();*/
  mutex_L[0] == 0; /*wait for lock*/
  mutex_L[0] = 1; /*lock*/
  d_step
  {
    ch_toL[0]!ADVANCE;
    mutex_L[0] = 0; /*unlock*/
  }
  mutex_L[1] == 0; /*wait for lock*/
  mutex_L[1] = 1; /*lock*/
  d_step
  {
    ch_toL[1]!ADVANCE;
    mutex_L[1] = 0; /*unlock*/
  }
  /*blockPedestrians();*/
  state_P[0] = DONT_WALK;
  ch_toL[0]!ALL_STOP;
  state_P[1] = DONT_WALK;
  ch_toL[1]!ALL_STOP;
  /*advanceTurnLights();*/
    mutex_T[0] == 0; /*wait for lock*/
  mutex_T[0] = 1; /*lock*/
  d_step
  {
    ch_toT[0]!ADVANCE;
    mutex_T[0] = 0; /*unlock*/
  }
  mutex_T[1] == 0; /*wait for lock*/
  mutex_T[1] = 1; /*lock*/
  d_step
  {
    ch_toT[1]!ADVANCE;
    mutex_T[1] = 0; /*unlock*/
  }
  /*unblockPedestrians();*/
  state_P[0] = WALK;
  state_P[1] = WALK;

}

proctype LinearLightSet (bit i) {

  do
  ::state_L[i] == RED ->
    ch_toL[i]?ADVANCE -> state_L[i] = GREEN; ch_toL[i]!PRE_STOP; state_P[i] = DONT_WALK;
  ::state_L[i] == ORANGE ->
    ch_toL[i]?STOP -> state_L[i] = RED; state_P[i] = WALK;
  ::state_L[i] == GREEN ->
    ch_toL[i]?PRE_STOP -> state_L[i] = ORANGE; ch_toL[i]!STOP; state_P[i] = DONT_WALK;
  ::state_L[i] == OFF ->
    ch_toL[i]?INIT -> state_L[i] = RED; state_P[i] = WALK;
  od
}

proctype TurnLightSet (bit i) {
  do
  ::state_T[i] == RED ->
    ch_toT[i]?ADVANCE -> state_T[i] = GREEN; ch_toT[i]!PRE_STOP; 
  ::state_T[i] == ORANGE ->
    ch_toT[i]?STOP -> state_T[i] = RED; 
  ::state_T[i] == GREEN ->
    ch_toT[i]?PRE_STOP -> state_T[i] = ORANGE; ch_toT[i]!STOP;
  ::state_T[i] == OFF ->
    ch_toT[i]?INIT -> state_T[i] = RED; 
  od
}
