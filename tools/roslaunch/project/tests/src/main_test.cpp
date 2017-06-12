#include <iostream>
#include <unistd.h>
#include <vector>
#include <sys/wait.h>
#include <signal.h>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>

bool pass_signal(char sig, int pid){
	FILE *pipe;
	char buf[128];
	memset(buf, 0, 127);
	std::string result = "";
	pipe = popen((("ps h --ppid ") + std::to_string(pid) + (" -o pid -o stat")).c_str(), "r");

	if(!pipe){
		std::cout << "Could not open pipe for output." << std::endl;
		pclose(pipe);
		return false;
	}

	memset(buf, 0, 127);
	std::vector<int> children;
	int i = 0, ipd = 0;
	std::string pd = "", state = "";
	std::vector<std::pair<int, std::string> > process;

	while(fgets(buf, 128, pipe) != NULL) {
		i = 0;
		pd = "";
		state = "";
		if(buf[0] == ' ')
			i++;
		while(buf[i] != ' '){
			pd += buf[i];
			i++;
		}
		ipd = stoi(pd);
		i++;
		while(buf[i] != '\0'){
			state += buf[i];
			i++;
			if(buf[i] == '\n') 
				i++;
		}
		memset(buf, 0, 127);
		process.push_back(std::make_pair(ipd, state));
	}	

	pclose(pipe);
	std::string st = (sig == 'T') ? "stopped" : "resumed";
	bool flag = false;
	if(st == "resumed")
		flag = true;
	for(auto& a : process)
		if(a.second[0] == sig || (a.second[0] == 'R' && flag))
			std::cout << "Process with PID " << a.first << " was " << st << ", its state is " << a.second[0] << std::endl;
		else{
			std::cout << "Something went wrong... I think you need to increase sleep time! ps aux table has not been updated yet! Please restart test program! PID = " << a.first
                      << std::endl;
			return false;
		}

	return true;
}

int main(int argc, char **argv) {
	FILE *pipe;
	char buf[128];
	memset(buf, 0, 127);
	std::string result = "";
	pipe = popen("ps aux | grep ros | grep '/usr/bin/python [/a-bA-Z0-9].* [a-zA-Z0-9]* [.a-zA-Z0-9]*launch' | awk '{print $2}'", "r");

	if(!pipe){
		std::cout << "Could not open pipe for output." << std::endl;
		pclose(pipe);
		return 0;
	}

	int pd = 0;
	if(fgets(buf, 128, pipe) != NULL)
	pd = atoi(buf);
	
	if(!pd){
		std::cout << "roslaunch has not been started yet!" << std::endl;
		return 0;
	}
	memset(buf, 0, 127);
	std::cout << "roslaunch started with PID " << pd << std::endl;
	pclose(pipe);
	pipe = popen(("pgrep -P " + std::to_string(pd)).c_str(), "r");

	if(!pipe){
		std::cout << "Could not open pipe for output." << std::endl;
		pclose(pipe);
		return 0;
	}

	std::cout << "List of all child PIDs for roslaunch:" << std::endl; 
	std::string ch_pid = ""; 
	int i = 0;
	while(fgets(buf, 128, pipe) != NULL){
		i = 0;
		ch_pid = "";
		while(buf[i] != '\0'){
			ch_pid += buf[i];
			i++;
			if(buf[i] == '\n') 
				i++;
		}	
		memset(buf, 0, 127);
		std::cout << ch_pid << std::endl; 
	}	
	std::cout << std::endl;
	
	std::cout << "Sending SIGSTOP to roslaunch" << std::endl;
	kill(pd, 19); //SIGSTOP
	std::this_thread::sleep_for(std::chrono::milliseconds(70));
	if (pass_signal('T', pd))
		std::cout << "All tests for SIGSTOP were passed successfully" << std::endl << std::endl;
	else
		std::cout << "Testcase for SIGSTOP failed!" << std::endl << std::endl;

	std::cout << "Sending SIGCONT to roslaunch" << std::endl;
	kill(pd, 18); //SIGCONT
	std::this_thread::sleep_for(std::chrono::milliseconds(70));
	if (pass_signal('S', pd))
		std::cout << "All tests for SIGCONT were passed successfully" << std::endl;
	else
		std::cout << "Testcase for SIGCONT failed!" << std::endl;
	
	if (!kill(pd, 0))
		kill(pd, 2);

	return 0;
}
