mtype = {OFF, RED, GREEN, ORANGE, WALK, DONT_WALK};
mtype = {INIT, ADVANCE, PRE_STOP, STOP, ALL_STOP};
mtype = {L, T};
mtype = {ACK};

chan ch_toT[2] = [1] of { mtype };
chan ch_toL[2] = [1] of { mtype };
chan ch_ack = [3] of { mtype, bit, mtype }; /*acknowledgement: lightType, lightId, action*/

/* Current state of lights. */
mtype state_L[2];
mtype state_T[2];

/* Pedestrian Light. */
mtype state_P[2];

/* Mutex for light sets. Used for synchronization. */

bit ltlTest;

/* LTL Statements */
/* Test */
ltl test {[](state_P[0] == WALK -> ltlTest == 1)}
/* when a pedestrian light is on WALK, the opposite vehicle stoplight must be RED */
ltl p1 {[](state_P[0] == WALK -> state_L[0] == RED)} /* changed murat -> passes */
ltl p2 {[](state_P[1] == WALK -> state_L[1] == RED)} /* changed murat -> passes */
/* when a pedestrian light is on WALK, all vehicle turn lights must be RED */
ltl p3 {[](state_P[0] == WALK -> (state_T[1] == RED && state_T[0] == RED))} /* passes */
ltl p4 {[](state_P[1] == WALK -> (state_T[1] == RED && state_T[0] == RED))} /* passes */
/* a pedestrian light is switched to WALK after the opposite vehicular lights have been switched to RED */
ltl p5 {[]((state_L[0] == RED) -> ((state_P[1] == DONT_WALK) && X(state_P[1] == WALK)))} /* fails during Initialization phase */
ltl p6 {[]((state_L[1] == RED) -> ((state_P[0] == DONT_WALK) && X(state_P[0] == WALK)))} /* fails during Initialization phase */
/* a pedestrian light is switched to DON’T WALK before the opposite vehicular lights are switched to GREEN */
ltl p7 {[](((state_P[1] == WALK) && X(state_P[1] == DONT_WALK)) -> (state_L[0] == GREEN))} /* fails during Initialization phase */
ltl p8 {[](((state_P[0] == WALK) && X(state_P[0] == DONT_WALK)) -> (state_L[1] == GREEN))} /* fails during Initialization phase */

/* incoming pedestrians from any direction can cross the intersection in that direction */
ltl p9 {(state_P[0] == DONT_WALK) -> []<>(state_P[0] == WALK)} /* passes but only 1 state stored? */
ltl p10 {(state_P[1] == DONT_WALK) -> []<>(state_P[1] == WALK)} /* passes but only 1 state stored? */

ltl p9b {[]((state_P[0] == DONT_WALK) -> <>(state_P[0] == WALK))} /* passes */
ltl p10b {(state_P[1] == DONT_WALK) -> []<>(state_P[1] == WALK)} /* passes */

/* incoming vehicles from any direction can cross the intersection in that direction */
ltl p11 {(state_L[0] == RED) -> []<>(state_L[0] == GREEN)} /* passes but only 1 state stored? */
ltl p12 {(state_L[0] == ORANGE) -> []<>(state_L[0] == GREEN)} /* passes but 1 state? */
ltl p13 {(state_L[1] == RED) -> []<>(state_L[1] == GREEN)} /* passes but 1 state? */
ltl p14 {(state_L[1] == ORANGE) -> []<>(state_L[1] == GREEN)} /* passes but 1 state? */

/* incoming vehicles from any direction can make a protected left turn */
ltl p15 {(state_T[0] == RED) -> []<>(state_T[0] == GREEN)} /* passes but 1 state? */
ltl p16 {(state_T[0] == ORANGE) -> []<>(state_T[0] == GREEN)} /* passes but 1 state? */
ltl p17 {(state_T[1] == RED) -> []<>(state_T[1] == GREEN)} /* passes but 1 state? */
ltl p18 {(state_T[1] == ORANGE) -> []<>(state_T[1] == GREEN)} /* passes but 1 state? */

/* For any vehicle light (stoplight or turn light), always: the signal eventually turns ORANGE */
ltl p19 {[]<>(state_L[0] == ORANGE)} /* passes */
ltl p20 {[]<>(state_L[1] == ORANGE)} /* passes */
ltl p21 {[]<>(state_T[0] == ORANGE)} /* passes */
ltl p22 {[]<>(state_T[1] == ORANGE)} /* passes */

/* For any vehicle light (stoplight or turn light), always: if a GREEN signal is on, 
it stays on until the signal turns ORANGE */
ltl p23 {[]((state_L[0] ==  GREEN) -> ((state_L[0] == GREEN) U (state_L[0] == ORANGE)))} /* passes */
ltl p24 {[]((state_L[1] ==  GREEN) -> ((state_L[1] == GREEN) U (state_L[1] == ORANGE)))} /* passes */
ltl p25 {[]((state_T[0] ==  GREEN) -> ((state_T[0] == GREEN) U (state_T[0] == ORANGE)))} /* passes */
ltl p26 {[]((state_T[1] ==  GREEN) -> ((state_T[1] == GREEN) U (state_T[1] == ORANGE)))} /* passes */

/* For any vehicle light (stoplight or turn light), 
always: if a RED signal is on, it stays on until the signal turns GREEN */
ltl p27 {[]((state_L[0] ==  RED) -> ((state_L[0] == RED) U (state_L[0] == GREEN)))} /* passes */
ltl p28 {[]((state_L[1] ==  RED) -> ((state_L[1] == RED) U (state_L[1] == GREEN)))} /* passes */
ltl p29 {[]((state_T[0] ==  RED) -> ((state_T[0] == RED) U (state_T[0] == GREEN)))} /* passes */
ltl p30 {[]((state_T[1] ==  RED) -> ((state_T[1] == RED) U (state_T[1] == GREEN)))} /* passes */




/* Client = Operator(of intersection controller) = Main */
/* A Kickstarter, where scheduling logic runs in its own thread */
proctype Operator() {
  ltlTest = 1;
  
  state_L[0] = OFF;
  state_L[1] = OFF;
  state_T[0] = OFF;
  state_T[1] = OFF;
  
  run LinearLightSet(0);
  run LinearLightSet(1);
  run TurnLightSet(0);
  run TurnLightSet(1);
  
  
  /*part of initialization*/
  ch_toT[0]!INIT;
  ch_ack ? T, 0, INIT;
  ch_toT[1]!INIT;
  ch_ack ? T, 1, INIT;
  
  ch_toL[0]!INIT;
  ch_ack ? L, 0, INIT;
  ch_toL[1]!INIT;
  ch_ack ? L, 1, INIT;
  
  run Scheduler();
}

proctype Scheduler() {
  
  replay:
  ch_toL[0] ! ADVANCE;
  ch_ack ? L, 0, ACK;
  
  ch_toL[1] ! ADVANCE;
  ch_ack? L, 1, ACK;
  
  ch_toL[0] ! ALL_STOP;
  ch_ack? L, 0, ALL_STOP;
  
  ch_toL[1] ! ALL_STOP;
  ch_ack? L, 1, ALL_STOP;
  
  ch_toT[0] ! ADVANCE;
  ch_ack ? T, 0, ACK;
  
  ch_toT[1] ! ADVANCE;
  ch_ack ? T, 1, ACK;
  goto replay;

}

proctype LinearLightSet (bit i) {
  do
  ::state_L[i] == RED -> if
  						 ::ch_toL[i]?ADVANCE -> atomic{ state_P[i] = DONT_WALK; state_L[i] = GREEN; ch_toL[i]!PRE_STOP; }
  						 ::ch_toL[i]?ALL_STOP -> atomic{ state_P[i] = DONT_WALK; ch_ack!L,i,ALL_STOP; }
  						 fi
  ::state_L[i] == ORANGE -> atomic{ ch_toL[i]?STOP -> state_L[i] = RED; state_P[i] = WALK; ch_ack!L,i,ACK; }
  ::state_L[i] == GREEN -> atomic{ ch_toL[i]?PRE_STOP -> state_L[i] = ORANGE; ch_toL[i]!STOP; }
  ::state_L[i] == OFF -> atomic{ ch_toL[i]?INIT -> state_P[i] = DONT_WALK; state_L[i] = RED; ch_ack!L,i,INIT; }
  od
}

proctype TurnLightSet (bit i) {
  do
  ::state_T[i] == RED -> atomic{ ch_toT[i]?ADVANCE -> state_T[i] = GREEN; ch_toT[i]!PRE_STOP; } 
  ::state_T[i] == ORANGE -> atomic{ ch_toT[i]?STOP -> state_T[i] = RED; ch_ack!T,i,ACK;} 
  ::state_T[i] == GREEN -> atomic{ ch_toT[i]?PRE_STOP -> state_T[i] = ORANGE; ch_toT[i]!STOP; }
  ::state_T[i] == OFF -> atomic{ ch_toT[i]?INIT -> state_T[i] = RED; ch_ack!T,i,INIT; } 
  od
}

init{
	run Operator();
}
