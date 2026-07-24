#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <time.h>
#include <semaphore.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include "common.h"

int main(){

	//Shared Memory
	int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
	ftruncate(shm_fd, sizeof(shared_data_t));
	shared_data_t* shared_data = mmap(0, sizeof(shared_data_t), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

	// --- SEMAPHORE SETUP ---
	    // Open a named semaphore. If it exists, it opens it. If not, it creates it.
	    // Initial value is 1 (unlocked).
	sem_unlink("/slam_sem");
	    sem_t *sem = sem_open("/slam_sem", O_CREAT, 0666, 1);
	    if (sem == SEM_FAILED) {
	        perror("sem_open failed");
	        return -1;
	    }

	//Reciever + Listener init
	int sockfd;
	char buffer[1024];
	struct sockaddr_in servaddr;

	//Watchdog + Latency init
	struct timespec last_packet_time, current_time;
	clock_gettime(CLOCK_MONOTONIC, &last_packet_time);

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	int opt = 1;
	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
		perror("setsockopt SO_REUSEADDR failed");
	    }

	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = INADDR_ANY;
	servaddr.sin_port = htons(5000);

	if (bind(sockfd, ( struct sockaddr *)&servaddr, sizeof(servaddr))<0){
		perror("Bind Failed");
		return -1;
	}

	struct timeval tv;
	tv.tv_sec = 1;  // 1 second timeout
	tv.tv_usec = 0;
	setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

	printf("QNX Slam listener Online. Waiting for drone Data...\n");

	//Boolean flag for watchdog
	int safe_mode = 0;

	while(1){

		int n = recvfrom(sockfd, buffer, 1024, 0, NULL, NULL);
		clock_gettime(CLOCK_MONOTONIC, &current_time);


		if (n>0){
			last_packet_time = current_time;
			if (safe_mode) {
				printf("[INFO]: Telemetry restored. Exiting Safe Mode.\n");
				safe_mode = 0;
					    }
			//clock_gettime(CLOCK_MONOTONIC, &last_packet_time);
			buffer[n] = '\0';

			// --- PARSER WITH SEMAPHORE ---
			    sem_wait(sem); // Request lock
			    sscanf(buffer, "%f, %f, %f", &shared_data->dist, &shared_data->x, &shared_data->y);
			    shared_data->updated = 1;
			    sem_post(sem); // Release lock

			//printf("[Telemetry Recieved]: %s | Latency: %ld ns\n", buffer, current_time.tv_nsec);

		}
		else{
			if ((current_time.tv_sec - last_packet_time.tv_sec)>3 && safe_mode == 0){
						printf("[WARNING!]: Telemetry Lost - Triggering Safe Mode..\n");
						//clock_gettime(CLOCK_MONOTONIC, &last_packet_time);
						last_packet_time = current_time;
						safe_mode = 1;
					}
		}

	}
	return 0;
}
