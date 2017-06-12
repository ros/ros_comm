#include <iostream>
#include <unistd.h>
#include <sys/wait.h>
#include <vector>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>

void pass_signal(int pid, int signal){
    FILE *pipe;
    char buf[128];
	memset(buf, 0, 127);
    std::string result = "";
    pipe = popen((("pgrep -P ") + std::to_string(pid)).c_str(), "r");

    if(!pipe){
        std::cout << "Could not open pipe for output." << std::endl;
        pclose(pipe);
        return;
    }

    std::vector<int> children;
    while(fgets(buf, 128, pipe) != NULL){
        children.push_back(stoi(std::string(buf)));
		memset(buf, 0, 127);
	}

    for(auto& a : children)
		kill(a, signal);
    pclose(pipe);

    return;
}

int main(int argc, char **argv) {
    pid_t cpid, w;
    int status;

    cpid = fork();
    if (cpid == -1) {
        std:: cout << "fork" << std::endl;
        return 1;
    }

    if (cpid == 0) {
        char *arr[] = {argv[1], argv[2], argv[3], NULL};
        execvp(argv[1], arr);
    }
    else {
        do {
            w = waitpid(cpid, &status, WUNTRACED | WCONTINUED);
            if (w == -1) {
                std::cout << "waitpid" << std::endl;
                break;
            }

            if (WIFEXITED(status)) {
                std::cout << "exited" << std::endl;
                break;
            }
            else if (WIFSTOPPED(status) && WSTOPSIG(status) == SIGSTOP) {
                std::cout << "stopped by signal" << std::endl;
                pass_signal((int) cpid, SIGSTOP);
            }
            else if (WIFCONTINUED(status)) {
                std::cout << "resumed by signal" << std::endl;
                pass_signal((int) cpid, SIGCONT);
            }
        } while (!WIFEXITED(status) && !WIFSIGNALED(status));

    }
    return 0;
}

