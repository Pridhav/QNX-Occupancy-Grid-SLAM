#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <semaphore.h>
#include <math.h>
#include "common.h"
#include "map_common.h"

// Function Prototypes (The compiler's "Forward Look")
void worldToMap(float x, float y, int *i, int *j);
void setCell(int i, int j, int8_t value);
void castRay(int x0, int y0, int x1, int y1);
void updateGrid(float obstacle_x, float obstacle_y);

// Now your actual function definitions can go anywhere below this

occupancy_map_t local_map;

//SLAM FUNCS
void worldToMap(float x, float y, int *i, int *j){
	*i = (int)((x - local_map.origin_x) / RESOLUTION) + MAP_SIZE/2;
	*j = (int)((y - local_map.origin_y)/ RESOLUTION) + MAP_SIZE/2;
	//printf("DEBUG: World(%.2f, %.2f) -> Grid(%d, %d)\n", x, y, *i, *j);
}

void setCell(int i, int j, int8_t value){
	if (i >= 0 && i < MAP_SIZE && j >= 0 && j < MAP_SIZE){
		if (value == FREE) local_map.grid[i][j] = FREE;
		else if (local_map.grid[i][j] < OCCUPIED) local_map.grid[i][j] += 10;
	}
}

void castRay (int x0, int y0, int x1, int y1){
	int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = dx + dy, e2;

	while(1){
		if (x0 == x1 && y0 == y1){
			setCell (x0, y0, OCCUPIED);
			break;
		}
		setCell(x0, y0, FREE);

		e2 = 2*err;
		if (e2 >= dy){err += dy; x0 += sx;}
		if (e2 <= dx){err += dx; y0 += sy;}
	}
}

void updateGrid(float obstacle_x, float obstacle_y){
	int i_start = MAP_SIZE / 2;
	int j_start = MAP_SIZE /2;
	int i_end, j_end;
	worldToMap(obstacle_x ,obstacle_y, &i_end, &j_end);
	castRay(i_start, j_start, i_end, j_end);
}

int main(){

	sleep(2);

	//SLAM INIT
	local_map.origin_x = 0.0f;
	local_map.origin_y = 0.0f;

	for(int i = 0; i < MAP_SIZE; i++){
		for (int j = 0; j < MAP_SIZE; j++){
			local_map.grid[i][j] = FREE;
		}
	}


	//Open Shared mem
	int shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
	if (shm_fd == -1){
		perror("shm_open failed - make sure listener is running!");
		return 1;
	}

	//printf("Before mmap\n");
	shared_data_t* shared_data = mmap(NULL, sizeof(shared_data_t), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
	printf("Engine connected to SHM. Waiting for telemetry...\n");

	if (shared_data == MAP_FAILED) {
		    perror("mmap failed");
		    return 1;
		}

	// FORCE A MEMORY READ TEST
	//printf("Pointer Address: %p\n", (void*)shared_data);
	//printf("Checking struct initial value: %d\n", shared_data->updated); // IF IT CRASHES HERE, THE POINTER IS BAD

	sem_t *sem = sem_open("/slam_sem", 0);
	if (sem == SEM_FAILED) {
	    perror("Engine could not open semaphore");
	    return 1;
	}

	//Poll for data
	while (1) {
		shared_data_t local_copy;
		    int has_data = 0;

		    sem_wait(sem);
		    if (shared_data->updated) {
		        local_copy = *shared_data;
		        shared_data->updated = 0;
		        has_data = 1;
		    }
		    sem_post(sem);

		    if (has_data) {
		        //printf("Read from SHM: Dist: %.2f | X: %.2f | Y: %.2f\n",local_copy.dist, local_copy.x, local_copy.y);
		    	updateGrid(local_copy.x, local_copy.y);

		    	static int frame_count = 0;
		    	if (++frame_count % 20 == 0){
		    		for(int i = 0; i < 30; i++) printf("\n");
		    		printf("\033[H");
		    		printf("\033[J");

		    		printf("----SLAM GRID (200X200) | Drone Tracking Active----\n");
		    		for (int i = 0; i < MAP_SIZE; i+=4){
		    			for (int j = 0; j < MAP_SIZE; j+=4){
		    				if (local_map.grid[i][j] >= 20) printf("#");
		    				else if (local_map.grid[i][j]>0) printf(":");
		    				else printf(".");
		    			}
		    			printf("\n");
		    		}
		    	}
		    } else {
		        // Heartbeat: This prints if we are checking but no new data is found
		         //printf(".");
		        // fflush(stdout);
		    }

		    usleep(50000); // 50ms
		}
	return 0;
}
