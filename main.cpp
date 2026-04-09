#include "utils.h"

extern void generator();
extern void* display_thread(void*);

static pid_t spawner_process = 0;
static pthread_t logger_thread_id = 0;

void cleanup_handler(int sig) {
    keep_running = false;
    
    if (spawner_process > 0) {
        kill(spawner_process, SIGTERM);
        waitpid(spawner_process, NULL, WNOHANG);
    }
    
    pthread_mutex_lock(&queue_lock);
    for (auto& aircraft : all_aircraft) {
        kill(aircraft.pid, SIGTERM);
        kill(aircraft.pid, SIGKILL);
    }
    pthread_mutex_unlock(&queue_lock);
    
    if (logger_thread_id != 0) {
        pthread_cancel(logger_thread_id);
    }
    
    write_to_log("ATC: Emergency shutdown complete.");
    close(comm_pipe[0]);
    close(comm_pipe[1]);
    fclose(output_log);
    
    exit(0);
}

int main() {
    sim_begin = time(NULL);
    output_log = fopen(LOG_FILE, "w");
    if (!output_log) {
        perror("fopen");
        return 1;
    }

    cout << "================================================================================\n";
    cout << "            OPERATION SKYWATCH - ATC COMMAND CONSOLE\n";
    cout << "================================================================================\n";
    cout << "Name: Abdul Rafay\n";
    cout << "Roll No: " << ROLL_NUMBER << "\n";
    cout << "Log File: " << LOG_FILE << "\n";
    cout << "================================================================================\n\n";
    
    cout << "Simulation starting...\n";
    cout << "All queue updates and statistics are logged to: " << LOG_FILE << "\n";
    cout << "Use 'tail -f " << LOG_FILE << "' in another terminal to watch updates.\n\n";
    
    cout << "Available Commands:\n";
    cout << "  force_emergency <id>       - Move jet to emergency queue\n";
    cout << "  new_jet <type> <fuel>      - Manually spawn new jet\n";
    cout << "  change_quantum <n>         - Change Round Robin quantum (1-20)\n";
    cout << "  boost_priority <id>        - Move jet to higher priority queue\n";
    cout << "  pause_sim                  - Pause/resume simulation\n";
    cout << "  status                     - Show detailed status\n";
    cout << "  exit                       - Shutdown simulation\n";
    cout << "================================================================================\n\n";
    
    signal(SIGINT, cleanup_handler);
    signal(SIGTERM, cleanup_handler);
    
    write_to_log("Simulation started.");
    
    int random_seed = 14;
    write_to_log("Seeding random generator with value: " + to_string(random_seed));
    srand(random_seed);
    
    if (pipe(comm_pipe) == -1) {
        perror("pipe");
        return 1;
    }
    
    spawner_process = fork();
    if (spawner_process == 0) {
        close(comm_pipe[0]);
        generator();
        return 0;
    } else if (spawner_process < 0) {
        perror("fork");
        return 1;
    }
    
    pthread_create(&logger_thread_id, NULL, display_thread, NULL);
    
    int fd_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, fd_flags | O_NONBLOCK);
    
    last_stat_check = sim_begin;
    
    string command_buffer;
    
    int aging_tick = 0;
    int refuel_tick = 0;
    
    while (keep_running) {
        char input_char;
        if (read(STDIN_FILENO, &input_char, 1) > 0) {
            if (input_char == '\n') {
                if (!command_buffer.empty()) {
                    parse_user_command(command_buffer.c_str());
                    command_buffer.clear();
                }
                cout << "ATC >> ";
                cout.flush();
            } else if (input_char == 127 || input_char == 8) {
                if (!command_buffer.empty()) {
                    command_buffer.pop_back();
                }
            } else if (input_char >= 32 && input_char <= 126) {
                command_buffer += input_char;
            }
        }
        
        handle_incoming_messages();
        
        if (!sim_frozen) {
            pick_next_jet();
        }
        
        aging_tick++;
        if (aging_tick >= 100) {
            check_for_aging();
            aging_tick = 0;
        }
        
        refuel_tick++;
        if (refuel_tick >= 50) {
            check_refuel_status();
            refuel_tick = 0;
        }
        
        usleep(10000);
    }
    
    write_to_log("ATC: Shutting down...");
    kill(spawner_process, SIGTERM);
    waitpid(spawner_process, NULL, 0);
    
    pthread_mutex_lock(&queue_lock);
    for (auto& aircraft : all_aircraft) {
        kill(aircraft.pid, SIGTERM);
        kill(aircraft.pid, SIGKILL);
        waitpid(aircraft.pid, NULL, WNOHANG);
    }
    pthread_mutex_unlock(&queue_lock);
    
    pthread_cancel(logger_thread_id);
    pthread_join(logger_thread_id, NULL);
    
    close(comm_pipe[0]);
    close(comm_pipe[1]);
    fclose(output_log);

    cout << "\nSimulation complete. Log saved to: " << LOG_FILE << "\n";
    return 0;
}
