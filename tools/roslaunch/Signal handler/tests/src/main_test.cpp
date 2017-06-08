#include <iostream>
#include <unistd.h>
#include <vector>
#include <sys/wait.h>
#include <signal.h>
#include <string>

bool pass_signal(char sig, int pid){
    FILE *pipe;
    char buf[128];
    std::string result = "";
    pipe = popen((("ps h --ppid ") + std::to_string(pid) + (" -o pid -o stat")).c_str(), "r");

    if(!pipe){
        std::cout << "Could not open pipe for output.\n";
        pclose(pipe);
        return false;
    }

    std::vector<int> children;
    int i = 0, ipd = 0;
    std::string pd = "", state = "";
    std::vector<std::pair<int, std::string> > process;

    while(fgets(buf, 128, pipe) != NULL) {
        i = 0;
        pd = "";
        state = "";
        while(buf[i] != ' '){
            pd += buf[i];
            i++;
        }
        ipd = stoi(pd);
        i++;
        while(buf[i] != '\0'){
            state += buf[i];
            i++;
        }
        process.push_back(std::make_pair(ipd, state));

    }
    pclose(pipe);
    std::string st = (sig == 'T') ? "stopped" : "resumed";
    for(auto a : process)
        if(a.second[0] == sig)
            std::cout << "Process with PID " << a.first << " was " << st << std::endl;
        else{
            std::cout << "Error! Child proccess with PID " << a.first
                      << " wasn't " << st << ". Its state is " << a.second;
            std::cout << "Testcase failed!";
            return false;
        }
    return true;
}

int main(int argc, char **argv) {
 	FILE *pipe;
    char buf[128];
    std::string result = "";
    pipe = popen("ps aux | grep ros | grep '/usr/bin/python [/a-bA-Z0-9].* [a-zA-Z0-9]* [.a-zA-Z0-9]*launch' | awk '{print $2}'", "r");

    if(!pipe){
        std::cout << "Could not open pipe for output.\n";
        pclose(pipe);
        return false;
    }
	int pd = 0;
	if(fgets(buf, 128, pipe) != NULL)
		pd = atoi(buf);
    pclose(pipe);
 	kill(pd, 19); //SIGSTOP
    if (pass_signal('T', pd))
        std::cout << "All tests for SIGSTOP passed successfully" << std::endl;
    else
        std::cout << "Testcase for SIGSTOP failed!" << std::endl;

    kill(pd, 18); //SIGCONT
    if (pass_signal('S', pd))
        std::cout << "All tests for SIGCONT passed successfully" << std::endl;
    else
        std::cout << "Testcase for SIGCONT failed!" << std::endl;
	
	if (!kill(pd, 0))
        kill(pd, 2);

    return 0;
}
