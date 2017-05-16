/*-------------------------------- Arctic Core ------------------------------
 * Copyright (C) 2013, ArcCore AB, Sweden, www.arccore.com.
 * Contact: <contact@arccore.com>
 * 
 * You may ONLY use this file:
 * 1)if you have a valid commercial ArcCore license and then in accordance with  
 * the terms contained in the written license agreement between you and ArcCore, 
 * or alternatively
 * 2)if you follow the terms found in GNU General Public License version 2 as 
 * published by the Free Software Foundation and appearing in the file 
 * LICENSE.GPL included in the packaging of this file or here 
 * <http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt>
 *-------------------------------- Arctic Core -----------------------------*/

/*
 * DESCRIPTION
 *   Hold all OSEK system hooks.
 */

/* ----------------------------[includes]------------------------------------*/

#include <stdint.h>
#include "Os.h"
#include <assert.h>

#include "Mcu.h"

#include "multicore_i.h"

/* ----------------------------[private define]------------------------------*/
#define ERROR_LOG_SIZE (OS_NUM_CORES * 2)

#define USE_LDEBUG_PRINTF	1
#include "debug.h"

/* ----------------------------[private macro]-------------------------------*/


/* ----------------------------[private typedef]-----------------------------*/

typedef struct ErrorEntry {
	StatusType 		error;
	OsErrorType     info;
	TaskType    	taskId;
	OsServiceIdType serviceId;
} ErrorEntryType;


typedef struct ErrorLog {
	int 			index;
	ErrorEntryType 	log[ERROR_LOG_SIZE];
} ErrorLogType;

struct LogBad {
	uint32_t param1;
	uint32_t param2;
	uint32_t param3;
	TaskType taskId;
	OsServiceIdType serviceId;
	StatusType error;
	void    *pc;
	uint32_t coreId;
} LogBadType;


/* ----------------------------[private function prototypes]-----------------*/
/* ----------------------------[private variables]---------------------------*/
ErrorLogType ErrorLog;
/* ----------------------------[private functions]---------------------------*/
/* ----------------------------[public functions]----------------------------*/

/**
 *
 * @param FatalError
 * @return
 */
ProtectionReturnType ProtectionHook( StatusType FatalError ) {
    (void)FatalError;
	printf("## ProtectionHook\n");
	return PRO_KILLAPPL;
}

/**
 * TODO: this is a global hook, we should not do applicaion/specific operations
 */
void StartupHook( void )
{
	CoreIDType coreId = GetCoreID();
	uint32_t sys_freq;

	if (OS_CORE_ID_MASTER == coreId) {
		sys_freq = McuE_GetSystemClock();
	//	LDEBUG_PRINTF("## StartupHook\n");


		LDEBUG_PRINTF("Sys clock %d Hz\n", sys_freq);
	}
	(void)sys_freq;
}

/**
 *
 * @param Error
 */
void ShutdownHook( StatusType error ) {
	LDEBUG_FPUTS("## ShutdownHook\n");
	(void)error;
	while(1) {
		//err = err;
		;
	}
}

/**
 *
 * @param error
 */
void ErrorHook( StatusType error ) {

	TaskType task;
	static struct LogBad LogBad[ERROR_LOG_SIZE];
	static uint8_t ErrorCount = 0;
	CoreIDType coreId = Os_GetCoreId();

#ifndef USE_LDEBUG_PRINTF
	printf("ErrorHook: %d\n", error);
#endif
	GetTaskID(&task);


	OsServiceIdType service = OSErrorGetServiceId();

	/* Grab the arguments to the functions
	 * This is the standard way, see 11.2 in OSEK spec
	 */
	switch(service) {
	case OSServiceId_SetRelAlarm:
	{
		// Read the arguments to the faulty functions...
		AlarmType alarm_id = OSError_SetRelAlarm_AlarmId;
		TickType increment = OSError_SetRelAlarm_Increment;
		TickType cycle = OSError_SetRelAlarm_Cycle;
		(void)alarm_id;
		(void)increment;
		(void)cycle;

		// ... Handle this some way.
		break;
	}
	/*
	 * The same pattern as above applies for all other OS functions.
	 * See Os.h for names and definitions.
	 */

	default:
		break;
	}

	LDEBUG_PRINTF("## ErrorHook err=%u task=%u service=%d p1=0x%x, p2=0x"
		      "%x p3=0x%x pc=0x%x coreId=%d\n", error, task, service,
		      os_error.param1, os_error.param2, os_error.param3,
		      os_error.pc, coreId);

	/* Log the errors in a buffer for later review */
	LogBad[ErrorCount].param1 = os_error.param1;
	LogBad[ErrorCount].param2 = os_error.param2;
	LogBad[ErrorCount].param3 = os_error.param3;
	LogBad[ErrorCount].serviceId = service;
	LogBad[ErrorCount].taskId = task;
	LogBad[ErrorCount].error = error;
	LogBad[ErrorCount].pc = os_error.pc;

	ErrorCount++;

	/* Keep compiler silent */
	(void)LogBad[ErrorCount].param1;

	// Stall if buffer is full.
	while (ErrorCount >= ERROR_LOG_SIZE)
	{
		int i;

		for (i = 0; i < ERROR_LOG_SIZE; i++) {
		LDEBUG_PRINTF("## ErrorHook err=%u taskId=%u serviceId=%d p1=0x%x, p2=0x"
			      "%x p3=0x%x pc=0x%x coreId=%d\n", LogBad[ErrorCount].error,
			      LogBad[ErrorCount].taskId, LogBad[ErrorCount].serviceId,
			      LogBad[ErrorCount].param1, LogBad[ErrorCount].param2,
			      LogBad[ErrorCount].param3, LogBad[ErrorCount].pc,
			      LogBad[ErrorCount].coreId);
		};

		ErrorCount = 0;
	};
}


/**
 *
 */
void PreTaskHook( void ) {
	StatusType rv;
	TaskType task;
	TaskStateType state;
	printf("prehook\r\n");
	rv = GetTaskID(&task);
	assert( rv == E_OK );
	LDEBUG_PRINTF("## PreTaskHook, taskid=%d\n",task);
	rv = GetTaskState(task,&state);
	assert( rv == E_OK );
	assert( state == TASK_STATE_RUNNING );
}

/**
 *
 */
void PostTaskHook( void ) {
	StatusType rv;
	TaskType task;
	TaskStateType state;
	printf("poshook\r\n");
	rv = GetTaskID(&task);
	assert( rv == E_OK );
	LDEBUG_PRINTF("## PostTaskHook, taskid=%d\n",task);
	rv = GetTaskState(task,&state);
	assert( rv == E_OK );
	assert( state == TASK_STATE_RUNNING );

}

