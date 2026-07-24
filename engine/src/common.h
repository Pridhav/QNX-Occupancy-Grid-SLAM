/*
 * common.h
 *
 *  Created on: 18-Apr-2026
 *      Author: Pridhav Krishna
 */

#ifndef SRC_COMMON_H_
#define SRC_COMMON_H_

//#include <pthread.h>

#define SHM_NAME "/slam_shared_mem"


#pragma pack(push,1)
typedef struct {
	float dist;
	float x;
	float y;
	int updated;
}shared_data_t;
#pragma pack(pop)



#endif /* SRC_COMMON_H_ */
