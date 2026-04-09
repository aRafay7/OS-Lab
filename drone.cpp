#include "utils.h"

volatile sig_atomic_t can_execute = 0;
volatile sig_atomic_t emergency_flag = 0;
volatile sig_atomic_t terminate_flag = 0;
volatile sig_atomic_t refueling_flag = 0;

void handle_continue_signal(int sig) {
    can_execute = 1;
    refueling_flag = 0;
}

void handle_terminate_signal(int sig) {
    terminate_flag = 1;
}

void run_jet(int id, int burst, const char* jet_type) {
    signal(SIGCONT, handle_continue_signal);
    signal(SIGTERM, handle_terminate_signal);
    signal(SIGINT, handle_terminate_signal);
    
    close(comm_pipe[0]);
    char message[100];
    snprintf(message, sizeof(message), "%d,%d,NEW,%d,%s", getpid(), id, burst, jet_type);
    write(comm_pipe[1], message, strlen(message));
    write(comm_pipe[1], "\n", 1);
    
    int remaining_fuel = burst;
    bool emergency_reported = false;
    bool has_refueled = false;
    
    while (remaining_fuel > 0 && !terminate_flag) {
        if (remaining_fuel == 3 && !has_refueled) {
            snprintf(message, sizeof(message), "%d,%d,REFUEL,0", getpid(), id);
            write(comm_pipe[1], message, strlen(message));
            write(comm_pipe[1], "\n", 1);
            has_refueled = true;
            
            refueling_flag = 1;
            while (refueling_flag && !terminate_flag) {
                pause();
            }
            
            remaining_fuel = FULL_TANK;
            emergency_reported = false;
            
            continue;
        }
        
        while (can_execute == 0 && !terminate_flag) {
            pause();
        }
        
        if (terminate_flag) break;
        
        can_execute = 0;
        
        sleep(2);
        remaining_fuel--;
        
        if (remaining_fuel == EMERGENCY_FUEL_LEVEL && !emergency_reported) {
            snprintf(message, sizeof(message), "%d,%d,LOWFUEL,0", getpid(), id);
            write(comm_pipe[1], message, strlen(message));
            write(comm_pipe[1], "\n", 1);
            emergency_reported = true;
        }
        
        snprintf(message, sizeof(message), "%d,%d,%s,%d", getpid(), id, 
                remaining_fuel == 0 ? "DONE" : "RUN", remaining_fuel);
        write(comm_pipe[1], message, strlen(message));
        write(comm_pipe[1], "\n", 1);
    }
    exit(0);
}

void generator() {
    signal(SIGTERM, SIG_DFL);
    signal(SIGINT, SIG_DFL);
    
    int aircraft_number = 3400;
    while (keep_running) {
        sleep(MIN_SPAWN_GAP + rand() % (MAX_SPAWN_GAP - MIN_SPAWN_GAP + 1));
        aircraft_number++;
        int fuel_level = MIN_BURST + rand() % (MAX_BURST - MIN_BURST + 1);
        
        const char* aircraft_model = AIRCRAFT_MODELS[rand() % NUM_AIRCRAFT_TYPES];
        
        pid_t new_process = fork();
        if (new_process == 0) run_jet(aircraft_number, fuel_level, aircraft_model);
        write_to_log("GENERATOR: Created new jet " + to_string(aircraft_number) + " (" + 
                    string(aircraft_model) + ", PID " + to_string(new_process) + ")");
    }
    exit(0);
}
