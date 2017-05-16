
/*
 * multicore.h
 *
 *  Created on: 1 apr 2011
 *      Author: Niclas
 */

#ifndef MULTICORE_H_
#define MULTICORE_H_

#if (OS_NUM_CORES > 1)

#include "application.h"
#include "sys.h"

typedef struct {
	OsServiceIdType op;
	uint32_t arg1;
	uint32_t arg2;
	uint32_t arg3;
	boolean opFinished;
	StatusType result;
} OsCoreMessageBoxType;

struct msgBoxRw {
	uint16_t rd;
	uint16_t wr;
};

#define MBOX_Q_LEN 32

extern volatile unsigned int qLock[OS_NUM_CORES];
extern OsCoreMessageBoxType OsMessageBoxQ[OS_NUM_CORES][MBOX_Q_LEN];
extern struct msgBoxRw MsgBoxRw[OS_NUM_CORES];

boolean Os_OnRunningCore(ObjectTypeType ObjectType, uint32_t objectId);
void Os_CoreNotificationInit(void);
StatusType Os_NotifyCore(CoreIDType coreId, OsServiceIdType op,
                         uint32_t arg1, uint32_t arg2, uint32_t arg3);

static inline CoreIDType Os_GetCoreId(void)
{
	return Os_ArchCoreId();
}

#else
#define Os_OnRunningCore(x,y) true
#endif


#endif /* MULTICORE_H_ */

