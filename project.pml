mtype = {OFF, RED, GREEN, ORANGE, WALK, DONT_WALK};
mtype = {INIT, ADVANCE, PRE_STOP, STOP, ALL_STOP};

chan ch_toT[2] = [1] of { mtype };
chan ch_toL[2] = [1] of { mtype };

/* Current state of lights. */
mtype state_L[2];
mtype state_T[2];

//record whether a cycle has been completed by Linear Light
bit cycleForLCompleted[2];

/* Pedestrian Light. */
mtype state_P[2];

/* Mutex for light sets. Used for synchronization. */
bit mutex_L[2];
bit mutex_T[2];

/* LTL Statements */
/* when a pedestrian light is on WALK, the opposite vehicle stoplight must be RED */
ltl p1 {[](state_P[0] == WALK -> state_L[1] == RED)}
ltl p2 {[](state_P[1] == WALK -> state_L[0] == RED)}
/* when a pedestrian light is on WALK, all vehicle turn lights must be RED */
ltl p3 {[](state_P[0] == WALK -> (state_T[1] == RED && state_T[0] == RED))}
ltl p4 {[](state_P[1] == WALK -> (state_T[1] == RED && state_T[0] == RED))}
/* a pedestrian light is switched to WALK after the opposite vehicular lights have been switched to RED */
ltl p5 {[]((state_L[0] == RED) -> ((state_P[1] == DONT_WALK) && X(state_P[1] == WALK)))}
ltl p6 {[]((state_L[1] == RED) -> ((state_P[0] == DONT_WALK) && X(state_P[0] == WALK)))}
/* a pedestrian light is switched to DON’T WALK before the opposite vehicular lights are switched to GREEN */
ltl p7 {[](((state_P[1] == WALK) && X(state_P[1] == DONT_WALK)) -> (state_L[0] == GREEN))}
ltl p8 {[](((state_P[0] == WALK) && X(state_P[0] == DONT_WALK)) -> (state_L[1] == GREEN))}

/* incoming pedestrians from any direction can cross the intersection in that direction */
ltl p9 {(state_P[0] == DONT_WALK) -> []<>(state_P[0] == WALK)}
ltl p10 {(state_P[1] == DONT_WALK) -> []<>(state_P[1] == WALK)}

/* incoming vehicles from any direction can cross the intersection in that direction */
ltl p11 {(state_L[0] == RED) -> []<>(state_L[0] == GREEN)}
ltl p12 {(state_L[0] == ORANGE) -> []<>(state_L[0] == GREEN)}
ltl p13 {(state_L[1] == RED) -> []<>(state_L[1] == GREEN)}
ltl p14 {(state_L[1] == ORANGE) -> []<>(state_L[1] == GREEN)}

/* incoming vehicles from any direction can make a protected left turn */
ltl p15 {(state_T[0] == RED) -> []<>(state_T[0] == GREEN)}
ltl p16 {(state_T[0] == ORANGE) -> []<>(state_T[0] == GREEN)}
ltl p17 {(state_T[1] == RED) -> []<>(state_T[1] == GREEN)}
ltl p18 {(state_T[1] == ORANGE) -> []<>(state_T[1] == GREEN)}

/* For any vehicle light (stoplight or turn light), always: the signal eventually turns ORANGE */
ltl p19 {[]<>(state_L[0] == ORANGE)}
ltl p20 {[]<>(state_L[1] == ORANGE)}
ltl p21 {[]<>(state_T[0] == ORANGE)}
ltl p22 {[]<>(state_T[1] == ORANGE)}

/* For any vehicle light (stoplight or turn light), always: if a GREEN signal is on, 
it stays on until the signal turns ORANGE */
ltl p21 {[]((state_L[0] ==  GREEN) -> ((state_L[0] == GREEN) U (state_L[0] == ORANGE)))}
ltl p22 {[]((state_L[1] ==  GREEN) -> ((state_L[1] == GREEN) U (state_L[1] == ORANGE)))}
ltl p23 {[]((state_T[0] ==  GREEN) -> ((state_T[0] == GREEN) U (state_T[0] == ORANGE)))}
ltl p24 {[]((state_T[1] ==  GREEN) -> ((state_T[1] == GREEN) U (state_T[1] == ORANGE)))}

/* For any vehicle light (stoplight or turn light), 
always: if a RED signal is on, it stays on until the signal turns GREEN */
ltl p25 {[]((state_L[0] ==  RED) -> ((state_L[0] == RED) U (state_L[0] == GREEN)))}
ltl p26 {[]((state_L[1] ==  RED) -> ((state_L[1] == RED) U (state_L[1] == GREEN)))}
ltl p27 {[]((state_T[0] ==  RED) -> ((state_T[0] == RED) U (state_T[0] == GREEN)))}
ltl p28 {[]((state_T[1] ==  RED) -> ((state_T[1] == RED) U (state_T[1] == GREEN)))}


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
  
  state_P[0] = DONT_WALK;
  state_P[1] = DONT_WALK;
  
  cycleForLCompleted[0]=false;
  cycleForLCompleted[1]=false;
  
  /*part of initialization*/
  
  /*turnOnStopLights*/
  mutex_L[0] = 0; /*wait for lock*/
  mutex_L[0] = 1; /*lock*/
  d_step{
    ch_toL[0]!INIT;
    mutex_L[0] = 0; /*unlock*/
  }
  cycleForLCompleted[0]==true;
  mutex_L[1] = 0; /*wait for lock*/
  mutex_L[1] = 1; /*lock*/
  d_step{
    ch_toL[1]!INIT;
    mutex_L[1] = 0; /*unlock*/
  }
    cycleForLCompleted[1]==true;
  
  /*turnOnTurnLights*/
  mutex_T[0] = 0; /*wait for lock*/
  mutex_T[0] = 1; /*lock*/
  d_step{
    ch_toT[0]!INIT;
    mutex_T[0] = 0; /*unlock*/
  }
  mutex_T[1] = 0; /*wait for lock*/
  mutex_T[1] = 1; /*lock*/
  d_step{
    ch_toT[1]!INIT;
    mutex_T[1] = 0; /*unlock*/
  }
  
  run Scheduler();
}

proctype Scheduler() {

  run LinearLightSet(0);
  run LinearLightSet(1);
  run TurnLightSet(0);
  run TurnLightSet(1);

	  /*advanceLinearLights();*/
	  replay: mutex_L[0] = 0; /*wait for lock*/
	  
	  mutex_L[0] = 1; /*lock*/
		cycleForLCompleted[0]=false;
	    ch_toL[0]!ADVANCE;
	    cycleForLCompleted[0]==true;

	    mutex_L[0] = 0; /*unlock*/
	  mutex_L[1] = 0; /*wait for lock*/
	  
	  mutex_L[1] = 1; /*lock*/

      //log("Going into d_step");
      	cycleForLCompleted[1]=false;
	    ch_toL[1]!ADVANCE;
		cycleForLCompleted[1]==true;
	    mutex_L[1] = 0; /*unlock*/
	  
	  
	  /*blockPedestrians();*/
	  atomic
	  {
	  state_P[0] = DONT_WALK;
	  state_P[1] = DONT_WALK;
	  }
    cycleForLCompleted[0]==true;
	  ch_toL[0]!ALL_STOP;
    cycleForLCompleted[1]==true;
	  ch_toL[1]!ALL_STOP;
	  
	  /*advanceTurnLights();*/
	  mutex_T[0] = 0; /*wait for lock*/
	  	  atomic{
	  mutex_T[0] = 1; /*lock*/

	    ch_toT[0]!ADVANCE;
	    mutex_T[0] = 0; /*unlock*/
	  }
	  mutex_T[1] = 0; /*wait for lock*/
	  	  atomic{
	  mutex_T[1] = 1; /*lock*/

	    ch_toT[1]!ADVANCE;
	    mutex_T[1] = 0; /*unlock*/
	  }
	  
	  /*unblockPedestrians();*/
	  atomic
	  {
	  state_P[0] = WALK;
	  state_P[1] = WALK;
	  }
   goto replay;

}

proctype LinearLightSet (bit i) {
  do
  ::state_L[i] == RED ->
  	::if
      ::ch_toL[i]?ADVANCE -> atomic{state_L[i] = GREEN; ch_toL[i]!PRE_STOP; state_P[i] = DONT_WALK};
	  ::ch_toL[i]?ALL_STOP -> atomic{state_L[i] = RED};
	  fi
	::else -> skip;
  ::state_L[i] == ORANGE -> atomic{ ch_toL[i]?STOP -> state_L[i] = RED; cycleForLCompleted[i]=true; }
  ::state_L[i] == GREEN ->
    atomic{ ch_toL[i]?PRE_STOP -> state_L[i] = ORANGE; ch_toL[i]!STOP; state_P[i] = DONT_WALK; }
  ::state_L[i] == OFF -> atomic{ ch_toL[i]?INIT -> state_L[i] = RED; state_P[i] = WALK; cycleForLCompleted[i]=true;}
  od
}

proctype TurnLightSet (bit i) {
  do
  ::state_T[i] == RED -> atomic{ ch_toT[i]?ADVANCE -> state_T[i] = GREEN; ch_toT[i]!PRE_STOP; } 
  ::state_T[i] == ORANGE -> atomic{ ch_toT[i]?STOP -> state_T[i] = RED; } 
  ::state_T[i] == GREEN -> atomic{ ch_toT[i]?PRE_STOP -> state_T[i] = ORANGE; ch_toT[i]!STOP; }
  ::state_T[i] == OFF -> atomic{ ch_toT[i]?INIT -> state_T[i] = RED; } 
  od
}

init{
	run Operator();
}
