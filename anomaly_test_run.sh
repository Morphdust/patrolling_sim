#!/bin/bash
#
# GIT version. Please do not edit this file.
# Make a copy and edit the copy.
#
# MAP: Available maps: grid, example, cumberland, broughton, DIAG_labs, DIAG_floor1
# NROBOTS: number of robots
# INITPOS: initial positions of the robots: (default,a,b,c,...)
# ALG: Algorithm: RAND,CR,HCR,HPCC,CGG,MSP,GBS,SEBS,CBLS,DTAG,DTAP
# LOC: Localization mode: AMCL, fake_localization
# NAV: Navigation module: ros, spqrel_navigation
# GWAIT: Goal wait: how much time the robot stops when it reaches a goal
# COMMDELAY: communication delay of all messages (in seconds)
# TERM: Terminal to use gnome-terminal,xterm
# TIMEOUT: simulation timeout (seconds)
# CUSTOM_STAGE: flag if custom version of Stage is used: true, false
# SPEEDUP: simulator speedup (if Custom Stage is used)

MAP=cumberland
NROBOTS=8
INITPOS=default
ALG=DTAP
LOC=AMCL
NAV=ros
GWAIT=0
COMMDELAY=0.2
TERM=gnome-terminal
TIMEOUT=180
CUSTOM_STAGE=true
SPEEDUP=1.0

if [[ $# -eq 1 ]] ;
then
  ALG=$1
else
    echo "No arguments passed, running default"
fi

./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP

# The command terminates after TIMEOUT. More instances of this command can be repeated for performing
# multiple batch experiments.
#
# E.g., the following script runs three experiments with different number of robots

export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
./src/patrolling_sim/start_experiment_screen.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP
export DISPLAY=:1
date
