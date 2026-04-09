#include "utils.h"

const char* AIRCRAFT_MODELS[NUM_AIRCRAFT_TYPES] = {"F-35", "F-15", "F-22", "F-16", "F-18", "B-2", "A-10"};

FILE* output_log;
time_t sim_begin;
int comm_pipe[2];
bool keep_running = true;
bool sim_frozen = false;
int current_time_slice = TIME_SLICE;

pthread_mutex_t queue_lock = PTHREAD_MUTEX_INITIALIZER;
priority_queue<Jet, deque<Jet>, greater<Jet>> emergency_q;
queue<Jet> standard_q;
queue<Jet> waiting_q;
vector<Jet> all_aircraft;
Jet* runway_jet = NULL;
int slice_remaining = TIME_SLICE;
int finished_count = 0;
int total_switches = 0;
double sum_turnaround = 0;
double sum_waiting = 0;
double sum_response = 0;
int cpu_busy_time = 0;
time_t last_stat_check;

void write_to_log(string s) {
    char timestamp[20];
    snprintf(timestamp, sizeof(timestamp), "[%08.0f] ", difftime(time(NULL), sim_begin));
    fprintf(output_log, "%s%s\n", timestamp, s.c_str());
    fflush(output_log);
}

Jet* locate_jet_by_pid(pid_t p) {
    for (auto& aircraft : all_aircraft) 
        if (aircraft.pid == p) return &aircraft;
    return NULL;
}

Jet* locate_jet_by_id(int id) {
    for (auto& aircraft : all_aircraft) 
        if (aircraft.id == id) return &aircraft;
    return NULL;
}

bool pull_from_queue(pid_t p, Jet* output) {
    // Check emergency queue first
    deque<Jet> temp_storage;
    while (!emergency_q.empty()) {
        if (emergency_q.top().pid == p) { 
            *output = emergency_q.top(); 
            emergency_q.pop(); 
            for(auto& x : temp_storage) emergency_q.push(x); 
            return true; 
        }
        temp_storage.push_back(emergency_q.top()); 
        emergency_q.pop();
    }
    for(auto& x : temp_storage) emergency_q.push(x);
    
    // Check standard queue (Q2)
    int q2_size = standard_q.size();
    for (int i = 0; i < q2_size; i++) {
        Jet current_jet = standard_q.front(); 
        standard_q.pop();
        if (current_jet.pid == p) { 
            *output = current_jet; 
            return true; 
        }
        standard_q.push(current_jet);
    }
    
    // Check waiting queue (Q3)
    int q3_size = waiting_q.size();
    for (int i = 0; i < q3_size; i++) {
        Jet current_jet = waiting_q.front(); 
        waiting_q.pop();
        if (current_jet.pid == p) { 
            *output = current_jet; 
            return true; 
        }
        waiting_q.push(current_jet);
    }
    return false;
}

void stop_current_jet() {
    if (!runway_jet) return;
    kill(runway_jet->pid, SIGSTOP);
    total_switches++;
    
    if (runway_jet->priority_level == 1) emergency_q.push(*runway_jet);
    else if (runway_jet->priority_level == 2) standard_q.push(*runway_jet);
    else waiting_q.push(*runway_jet);
    
    delete runway_jet;
    runway_jet = NULL;
}

void pick_next_jet() {
    pthread_mutex_lock(&queue_lock);
    
    if (sim_frozen) {
        pthread_mutex_unlock(&queue_lock);
        return;
    }
    
    if (runway_jet) {
        pthread_mutex_unlock(&queue_lock);
        return;
    }
    
    Jet selected;
    bool found_one = false;
    
    if (!emergency_q.empty()) { 
        selected = emergency_q.top(); 
        emergency_q.pop(); 
        found_one = true; 
    } else if (!standard_q.empty()) { 
        selected = standard_q.front(); 
        standard_q.pop(); 
        slice_remaining = current_time_slice;
        found_one = true; 
    } else if (!waiting_q.empty()) { 
        selected = waiting_q.front(); 
        waiting_q.pop(); 
        found_one = true; 
    }
    
    if (found_one) {
        runway_jet = new Jet(selected);
        total_switches++;
        
        if (!runway_jet->started_running) {
            runway_jet->start_exec_time = time(NULL);
            runway_jet->started_running = true;
            
            Jet* jet_ptr = locate_jet_by_pid(runway_jet->pid);
            if (jet_ptr) {
                jet_ptr->started_running = true;
                jet_ptr->start_exec_time = runway_jet->start_exec_time;
            }
        }
        
        write_to_log("RUNWAY: Cleared for Jet " + to_string(runway_jet->id) + 
                     " (" + string(runway_jet->model) + ")");
        kill(runway_jet->pid, SIGCONT);
    }
    
    pthread_mutex_unlock(&queue_lock);
}

void check_for_aging() {
    pthread_mutex_lock(&queue_lock);
    
    time_t current_time = time(NULL);
    queue<Jet> temp_waiting;
    vector<Jet> jets_to_promote;
    
    while (!waiting_q.empty()) {
        Jet aircraft = waiting_q.front();
        waiting_q.pop();
        
        Jet* main_jet = locate_jet_by_pid(aircraft.pid);
        if (main_jet && main_jet->priority_level == 3) {
            main_jet->wait_counter++;
            aircraft.wait_counter = main_jet->wait_counter;
        }
        
        if (aircraft.wait_counter >= Q3_AGING_TIME) {
            aircraft.priority_level = 2;
            aircraft.wait_counter = 0;
            jets_to_promote.push_back(aircraft);
            
            if (main_jet) {
                main_jet->priority_level = 2;
                main_jet->wait_counter = 0;
            }
            
            write_to_log("ATC: AGING PROMOTION Jet " + to_string(aircraft.id) + 
                        " (" + string(aircraft.model) + ") Q3 -> Q2");
        } else {
            temp_waiting.push(aircraft);
        }
    }
    
    waiting_q = temp_waiting;
    for (auto& aircraft : jets_to_promote) {
        standard_q.push(aircraft);
    }
    
    pthread_mutex_unlock(&queue_lock);
}

void handle_incoming_messages() {
    fcntl(comm_pipe[0], F_SETFL, O_NONBLOCK);
    char buffer[4096];
    int bytes_read = read(comm_pipe[0], buffer, sizeof(buffer)-1);
    if (bytes_read <= 0) return;
    buffer[bytes_read] = 0;
    
    char* message_line = strtok(buffer, "\n");
    while (message_line) {
        int process_id, jet_number, data_value;
        char msg_type[20];
        char aircraft_model[20];
        
        int fields_parsed = sscanf(message_line, "%d,%d,%[^,],%d,%s", 
                                   &process_id, &jet_number, msg_type, &data_value, aircraft_model);
        
        if (fields_parsed >= 4) {
            pthread_mutex_lock(&queue_lock);
            
            if (strcmp(msg_type, "NEW") == 0) {
                // New jet arrival
                const char* model_to_use = (fields_parsed == 5) ? aircraft_model : "F-35";
                Jet new_aircraft(process_id, jet_number, data_value, model_to_use);
                all_aircraft.push_back(new_aircraft);
                standard_q.push(new_aircraft);
                write_to_log("ATC: NEW INBOUND Jet " + to_string(jet_number) + " (" + string(model_to_use) + 
                           ", PID " + to_string(process_id) + ", Burst=" + to_string(data_value) + ") -> Q2");
            } else {
                Jet* aircraft_ptr = locate_jet_by_pid(process_id);
                if (aircraft_ptr) {
                    if (strcmp(msg_type, "LOWFUEL") == 0) {
                        // Emergency - move to Q1
                        Jet temp_jet;
                        if (runway_jet && runway_jet->pid == process_id) { 
                            runway_jet->priority_level = 1; 
                            aircraft_ptr->priority_level = 1; 
                        } else if (pull_from_queue(process_id, &temp_jet)) { 
                            temp_jet.priority_level = 1; 
                            emergency_q.push(temp_jet); 
                            *aircraft_ptr = temp_jet; 
                            if (runway_jet && runway_jet->fuel_left > temp_jet.fuel_left) 
                                stop_current_jet(); 
                        }
                        write_to_log("ATC: EMERGENCY Jet " + to_string(jet_number) + " (" + 
                                   string(aircraft_ptr->model) + ", low fuel) -> Q1");
                        
                    } else if (strcmp(msg_type, "DONE") == 0) {
                        // Jet finished
                        finished_count++;
                        cpu_busy_time += aircraft_ptr->fuel_amount;
                        
                        int turnaround = time(NULL) - aircraft_ptr->entry_time;
                        int wait_time = turnaround - aircraft_ptr->fuel_amount;
                        int response = aircraft_ptr->started_running ? 
                                      (aircraft_ptr->start_exec_time - aircraft_ptr->entry_time) : 0;
                        
                        sum_turnaround += turnaround; 
                        sum_waiting += wait_time;
                        sum_response += response;
                        
                        write_to_log("ATC: LANDING SUCCESSFUL for Jet " + to_string(jet_number) + " (" + 
                                   string(aircraft_ptr->model) + 
                                   ") [TAT=" + to_string(turnaround) + "s, WT=" + to_string(wait_time) + 
                                   "s, RT=" + to_string(response) + "s]");
                        
                        if (runway_jet && runway_jet->pid == process_id) { 
                            delete runway_jet; 
                            runway_jet = NULL; 
                        }
                        for (int i = 0; i < all_aircraft.size(); i++) 
                            if (all_aircraft[i].pid == process_id) { 
                                all_aircraft.erase(all_aircraft.begin() + i); 
                                break; 
                            }
                    
                    } else if (strcmp(msg_type, "REFUEL") == 0) {
                        // Refuel request
                        aircraft_ptr->at_fuel_station = true;
                        aircraft_ptr->refuel_began = time(NULL);
                        
                        // Remove from execution
                        if (runway_jet && runway_jet->pid == process_id) {
                            write_to_log("ATC: REFUEL REQUEST - Jet " + to_string(jet_number) + " (" + 
                                       string(aircraft_ptr->model) + ") entering refuel bay");
                            delete runway_jet;
                            runway_jet = NULL;
                        } else {
                            Jet temp_jet;
                            pull_from_queue(process_id, &temp_jet);
                            write_to_log("ATC: REFUEL REQUEST - Jet " + to_string(jet_number) + " (" + 
                                       string(aircraft_ptr->model) + ") entering refuel bay");
                        }
                            
                    } else if (strcmp(msg_type, "RUN") == 0) {
                        // Jet executed one cycle
                        if (runway_jet && runway_jet->pid == process_id) {
                            runway_jet->fuel_left = data_value; 
                            aircraft_ptr->fuel_left = data_value;
                            write_to_log("[UAV-" + string(ROLL_NUMBER) + "] (PID " + to_string(process_id) + 
                                       ") reporting fuel=" + to_string(data_value));
                            
                            if (runway_jet->priority_level == 2) {
                                slice_remaining--;
                                if (slice_remaining <= 0) { 
                                    runway_jet->priority_level = 3; 
                                    aircraft_ptr->priority_level = 3; 
                                    aircraft_ptr->wait_counter = 0;
                                    write_to_log("ATC: Jet " + to_string(jet_number) + " (" + 
                                               string(aircraft_ptr->model) + ") quantum expired -> Q3"); 
                                    stop_current_jet(); 
                                }
                            }
                            
                            // Allow jet to continue
                            if (runway_jet && runway_jet->pid == process_id) {
                                kill(runway_jet->pid, SIGCONT);
                            }
                        }
                    }
                }
            }
            pthread_mutex_unlock(&queue_lock);
        }
        message_line = strtok(NULL, "\n");
    }
}

void* display_thread(void*) {
    while (keep_running) {
        pthread_mutex_lock(&queue_lock);
        
        time_t current_moment = time(NULL);
        double time_elapsed = difftime(current_moment, sim_begin);
        
        char status_line[1024];
        
        snprintf(status_line, sizeof(status_line), "STATUS UPDATE [T=%08.0f] %s", 
                time_elapsed, sim_frozen ? "[PAUSED]" : "");
        write_to_log(status_line);
        
        if (runway_jet) {
            snprintf(status_line, sizeof(status_line), 
                    "  Runway: Jet %d (%s) Q%d Fuel=%d/%d", 
                    runway_jet->id, runway_jet->model, runway_jet->priority_level, 
                    runway_jet->fuel_left, runway_jet->fuel_amount);
            write_to_log(status_line);
        } else {
            write_to_log("  Runway: CLEAR");
        }
        
        snprintf(status_line, sizeof(status_line), 
                "  Q1: %lu jets | Q2: %lu jets | Q3: %lu jets", 
                emergency_q.size(), standard_q.size(), waiting_q.size());
        write_to_log(status_line);
        
        int refueling_jets = 0;
        for (auto& aircraft : all_aircraft) {
            if (aircraft.at_fuel_station) refueling_jets++;
        }
        if (refueling_jets > 0) {
            snprintf(status_line, sizeof(status_line), "  Refueling: %d jets", refueling_jets);
            write_to_log(status_line);
        }
        
        snprintf(status_line, sizeof(status_line), 
                "  Stats: Completed=%d | Switches=%d | Active=%lu", 
                finished_count, total_switches, all_aircraft.size());
        write_to_log(status_line);
        
        if (finished_count > 0 && time_elapsed > 0) {
            double cpu_percent = (cpu_busy_time / time_elapsed) * 100.0;
            if (cpu_percent > 100.0) cpu_percent = 100.0;
            snprintf(status_line, sizeof(status_line), 
                    "  CPU Util: %.2f%% | CS Rate: %.3f/s", 
                    cpu_percent, total_switches / time_elapsed);
            write_to_log(status_line);
        }
        
        pthread_mutex_unlock(&queue_lock);
        usleep(2000000);
    }
    return NULL;
}

void check_refuel_status() {
    pthread_mutex_lock(&queue_lock);
    
    time_t right_now = time(NULL);
    for (auto& aircraft : all_aircraft) {
        if (aircraft.at_fuel_station) {
            int time_refueling = right_now - aircraft.refuel_began;
            if (time_refueling >= REFUEL_DURATION) {
                // Refuel done
                aircraft.at_fuel_station = false;
                aircraft.fuel_left = FULL_TANK;
                aircraft.fuel_amount = FULL_TANK;
                aircraft.priority_level = 1;  // Emergency queue after refuel
                aircraft.wait_counter = 0;
                emergency_q.push(aircraft);
                
                write_to_log("ATC: REFUEL COMPLETE Jet " + to_string(aircraft.id) + " (" + 
                           string(aircraft.model) + ") refueled to " + to_string(FULL_TANK) + " -> Q1");
                
                // Wake up jet process
                kill(aircraft.pid, SIGCONT);
            }
        }
    }
    
    pthread_mutex_unlock(&queue_lock);
}

void push_to_emergency(int jet_id) {
    pthread_mutex_lock(&queue_lock);
    
    Jet* aircraft_ptr = locate_jet_by_id(jet_id);
    if (!aircraft_ptr) {
        pthread_mutex_unlock(&queue_lock);
        cout << "[ERROR] Jet " << jet_id << " not found!\n";
        return;
    }
    
    Jet temp_jet;
    if (runway_jet && runway_jet->id == jet_id) {
        runway_jet->priority_level = 1;
        aircraft_ptr->priority_level = 1;
        stop_current_jet();
    } else if (pull_from_queue(aircraft_ptr->pid, &temp_jet)) {
        temp_jet.priority_level = 1;
        emergency_q.push(temp_jet);
        *aircraft_ptr = temp_jet;
        if (runway_jet && runway_jet->fuel_left > temp_jet.fuel_left) {
            stop_current_jet();
        }
    }
    
    write_to_log("ATC: MANUAL OVERRIDE - Jet " + to_string(jet_id) + " forced to EMERGENCY (Q1)");
    cout << "[OK] Jet " << jet_id << " moved to Emergency Queue (Q1)\n";
    
    pthread_mutex_unlock(&queue_lock);
}

void spawn_new_aircraft(const char* type, int burst) {
    pthread_mutex_lock(&queue_lock);
    
    int next_number = 3400;
    for (auto& aircraft : all_aircraft) {
        if (aircraft.id >= next_number) next_number = aircraft.id + 1;
    }
    
    pid_t new_pid = fork();
    if (new_pid == 0) {
        extern void run_jet(int id, int burst, const char* jet_type);
        run_jet(next_number, burst, type);
        exit(0);
    } else {
        write_to_log("ATC: MANUAL SPAWN - Created new jet " + to_string(next_number) + " (" + string(type) + 
                   ", PID " + to_string(new_pid) + ", Burst=" + to_string(burst) + ")");
        cout << "[OK] New jet " << next_number << " (" << type << ") created with fuel=" << burst << "\n";
    }
    
    pthread_mutex_unlock(&queue_lock);
}

void adjust_time_slice(int new_val) {
    pthread_mutex_lock(&queue_lock);
    
    if (new_val < 1 || new_val > 20) {
        pthread_mutex_unlock(&queue_lock);
        cout << "[ERROR] Quantum must be between 1 and 20\n";
        return;
    }
    
    int old_val = current_time_slice;
    current_time_slice = new_val;
    
    if (runway_jet && runway_jet->priority_level == 2) {
        slice_remaining = new_val;
    }
    
    write_to_log("ATC: QUANTUM CHANGE - Changed from " + to_string(old_val) + " to " + to_string(new_val));
    cout << "[OK] Round Robin quantum changed from " << old_val << " to " << new_val << "\n";
    
    pthread_mutex_unlock(&queue_lock);
}

void move_jet_up(int jet_id) {
    pthread_mutex_lock(&queue_lock);
    
    Jet* aircraft_ptr = locate_jet_by_id(jet_id);
    if (!aircraft_ptr) {
        pthread_mutex_unlock(&queue_lock);
        cout << "[ERROR] Jet " << jet_id << " not found!\n";
        return;
    }
    
    int previous_level = aircraft_ptr->priority_level;
    
    Jet temp_jet;
    if (runway_jet && runway_jet->id == jet_id) {
        if (runway_jet->priority_level > 1) {
            runway_jet->priority_level--;
            aircraft_ptr->priority_level--;
            stop_current_jet();
        }
    } else if (pull_from_queue(aircraft_ptr->pid, &temp_jet)) {
        if (temp_jet.priority_level > 1) {
            temp_jet.priority_level--;
            temp_jet.wait_counter = 0;
            if (temp_jet.priority_level == 1) emergency_q.push(temp_jet);
            else if (temp_jet.priority_level == 2) standard_q.push(temp_jet);
            *aircraft_ptr = temp_jet;
        } else {
            // Already in Q1
            emergency_q.push(temp_jet);
        }
    }
    
    write_to_log("ATC: PRIORITY BOOST - Jet " + to_string(jet_id) + " boosted from Q" + 
                to_string(previous_level) + " to Q" + to_string(aircraft_ptr->priority_level));
    cout << "[OK] Jet " << jet_id << " boosted from Q" << previous_level << " to Q" << 
            aircraft_ptr->priority_level << "\n";
    
    pthread_mutex_unlock(&queue_lock);
}

void toggle_pause() {
    sim_frozen = !sim_frozen;
    
    if (sim_frozen) {
        write_to_log("ATC: SIMULATION PAUSED");
        cout << "[PAUSED] Simulation paused. Type 'pause_sim' again to resume.\n";
        
        if (runway_jet) {
            kill(runway_jet->pid, SIGSTOP);
        }
    } else {
        write_to_log("ATC: SIMULATION RESUMED");
        cout << "[RESUMED] Simulation resumed.\n";
        
        if (runway_jet) {
            kill(runway_jet->pid, SIGCONT);
        }
    }
}

void print_current_status() {
    pthread_mutex_lock(&queue_lock);
    
    cout << "\n================ SYSTEM STATUS ================\n";
    cout << "Time: " << difftime(time(NULL), sim_begin) << "s";
    cout << " | Status: " << (sim_frozen ? "PAUSED" : "RUNNING") << "\n";
    cout << "Quantum: " << current_time_slice;
    cout << " | Completed: " << finished_count;
    cout << " | Switches: " << total_switches << "\n";
    cout << "Active Jets: " << all_aircraft.size() << " (Q1:" << emergency_q.size() 
         << " Q2:" << standard_q.size() << " Q3:" << waiting_q.size() << ")\n";
    
    if (runway_jet) {
        cout << "Runway: Jet " << runway_jet->id << " (" << runway_jet->model 
             << ") Q" << runway_jet->priority_level << " Fuel:" << runway_jet->fuel_left 
             << "/" << runway_jet->fuel_amount << "\n";
    } else {
        cout << "Runway: CLEAR\n";
    }
    
    if (finished_count > 0) {
        cout << "Avg TAT:" << (sum_turnaround / finished_count) 
             << "s WT:" << (sum_waiting / finished_count) 
             << "s RT:" << (sum_response / finished_count) << "s\n";
    }
    
    cout << "===============================================\n";
    
    pthread_mutex_unlock(&queue_lock);
}

void parse_user_command(const char* cmd) {
    if (strncmp(cmd, "force_emergency ", 16) == 0) {
        int jet_id = atoi(cmd + 16);
        push_to_emergency(jet_id);
    }
    else if (strncmp(cmd, "new_jet ", 8) == 0) {
        char aircraft_type[20];
        int fuel_val;
        if (sscanf(cmd + 8, "%s %d", aircraft_type, &fuel_val) == 2) {
            spawn_new_aircraft(aircraft_type, fuel_val);
        } else {
            cout << "[ERROR] Usage: new_jet <type> <fuel>\n";
        }
    }
    else if (strncmp(cmd, "change_quantum ", 15) == 0) {
        int new_quantum = atoi(cmd + 15);
        adjust_time_slice(new_quantum);
    }
    else if (strncmp(cmd, "boost_priority ", 15) == 0) {
        int jet_id = atoi(cmd + 15);
        move_jet_up(jet_id);
    }
    else if (strcmp(cmd, "pause_sim") == 0) {
        toggle_pause();
    }
    else if (strcmp(cmd, "status") == 0) {
        print_current_status();
    }
    else if (strcmp(cmd, "exit") == 0) {
        keep_running = false;
        cout << "[EXIT] Shutting down...\n";
    }
    else if (strcmp(cmd, "help") == 0) {
        cout << "\nAvailable Commands:\n";
        cout << "  force_emergency <id>   - Force jet to emergency queue\n";
        cout << "  new_jet <type> <fuel>  - Manually spawn new jet\n";
        cout << "  change_quantum <n>     - Change RR quantum (1-20)\n";
        cout << "  boost_priority <id>    - Move jet to higher queue\n";
        cout << "  pause_sim              - Pause/resume simulation\n";
        cout << "  status                 - Show system status\n";
        cout << "  help                   - Show this help\n";
        cout << "  exit                   - Shutdown simulation\n\n";
    }
    else {
        cout << "[ERROR] Unknown command. Type 'help' for commands.\n";
    }
}
