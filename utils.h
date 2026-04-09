#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <queue>
#include <deque>
#include <vector>
#include <pthread.h>
#include <unistd.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

using namespace std;

#define ROLL_NUMBER "23i-2034"
#define LOG_FILE "23i-2034_skywatch_log.txt"
#define TIME_SLICE 4
#define EMERGENCY_FUEL_LEVEL 10
#define MIN_SPAWN_GAP 3
#define MAX_SPAWN_GAP 8
#define MIN_BURST 5
#define MAX_BURST 25
#define Q3_AGING_TIME 15 
#define REFUEL_DURATION 5
#define FULL_TANK 25

#define NUM_AIRCRAFT_TYPES 7
extern const char* AIRCRAFT_MODELS[NUM_AIRCRAFT_TYPES];

struct Jet {
    pid_t pid;
    int id, fuel_amount, fuel_left, priority_level, wait_counter;
    time_t entry_time;
    time_t start_exec_time;
    bool started_running;
    bool at_fuel_station;
    time_t refuel_began;
    char model[10];
    
    Jet(pid_t p=0, int i=0, int b=0, const char* t="F-35") : 
        pid(p), id(i), fuel_amount(b), fuel_left(b), priority_level(2), wait_counter(0),
        entry_time(time(NULL)), start_exec_time(0), started_running(false),
        at_fuel_station(false), refuel_began(0) {
        strncpy(model, t, sizeof(model)-1);
        model[sizeof(model)-1] = '\0';
    }
    
    bool operator>(const Jet& other) const { 
        return fuel_left > other.fuel_left; 
    }
};

extern FILE* output_log;
extern time_t sim_begin;
extern int comm_pipe[2];
extern bool keep_running;
extern bool sim_frozen;
extern int current_time_slice;

extern pthread_mutex_t queue_lock;
extern priority_queue<Jet, deque<Jet>, greater<Jet>> emergency_q;
extern queue<Jet> standard_q;
extern queue<Jet> waiting_q;
extern vector<Jet> all_aircraft;
extern Jet* runway_jet;
extern int slice_remaining;
extern int finished_count;
extern int total_switches;
extern double sum_turnaround;
extern double sum_waiting;
extern double sum_response;
extern int cpu_busy_time;
extern time_t last_stat_check;

void write_to_log(string s);
Jet* locate_jet_by_pid(pid_t p);
Jet* locate_jet_by_id(int id);
bool pull_from_queue(pid_t p, Jet* out);
void stop_current_jet();
void pick_next_jet();
void handle_incoming_messages();
void check_for_aging();
void check_refuel_status();

void push_to_emergency(int jet_id);
void spawn_new_aircraft(const char* type, int burst);
void adjust_time_slice(int new_val);
void move_jet_up(int jet_id);
void toggle_pause();
void print_current_status();
void parse_user_command(const char* cmd);

#endif
