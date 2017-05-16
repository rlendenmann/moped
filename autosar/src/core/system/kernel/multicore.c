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
#include "kernel.h"
#include "arch.h"
#include "application.h"
#include "Os_Cfg.h"
#include "multicore_i.h"
#include "Ioc.h"

volatile unsigned int qLock[OS_NUM_CORES];
OsCoreMessageBoxType OsMessageBoxQ[OS_NUM_CORES][MBOX_Q_LEN];
struct msgBoxRw MsgBoxRw[OS_NUM_CORES];

void StartCore(CoreIDType CoreID, StatusType* Status)
{
	if (CoreID >= OS_NUM_CORES) {
		*Status = E_OS_ID;
	}
	else if (Os_Sys[CoreID].status.os_started) {
		*Status = E_OS_ACCESS;
	}
	else if (Os_Sys[CoreID].status.activated) {
		*Status = E_OS_STATE;
	} else {
		boolean validId = Os_StartCore(CoreID);
		if (!validId) {
			*Status = E_OS_ID;
		} else {
			Os_Sys[CoreID].status.activated = true;
			*Status = E_OK;
		}
	}
}

boolean Os_OnRunningCore(ObjectTypeType ObjectType, uint32_t objectId)
{
	ApplicationType app = CheckObjectOwnership(ObjectType,objectId);

	if (Os_ApplGetCore(app) == GetCoreID()) {
		return true;
	}
	else {
		return false;
	}
}

CoreIDType GetCoreID(void)
{
	return Os_GetCoreId();
}

void Os_CoreNotificationInit(void)
{
	CoreIDType core_id = Os_GetCoreId();

	/* reset mailbox queue rd/wr */
	MsgBoxRw[core_id].rd = 0;
	MsgBoxRw[core_id].wr = 0;
	Os_ArchReleaseSpinlock(&qLock[core_id]);

	Os_ArchCoreNotificationInit();
}

StatusType Os_NotifyCore(CoreIDType coreId, OsServiceIdType op,
                         uint32_t arg1, uint32_t arg2, uint32_t arg3)
{
	if (coreId >= OS_NUM_CORES) {
		return E_OS_ID;
	}
	else if (!Os_Sys[coreId].status.os_started) {
		return E_OS_ACCESS;
	}
	else if (!Os_Sys[coreId].status.activated) {
		return E_OS_STATE;
	} else {
		uint16_t wr;

		Os_ArchGetSpinlock(&qLock[coreId]);
		wr = MsgBoxRw[coreId].wr;
		/* simple full check */
		if (OsMessageBoxQ[coreId][wr].result != E_OK) {
			Os_ArchReleaseSpinlock(&qLock[coreId]);
			return E_OS_RESOURCE;
		}
		if ((wr + 1) < MBOX_Q_LEN) {
			MsgBoxRw[coreId].wr = wr + 1;
		} else {
			MsgBoxRw[coreId].wr = 0;
		}

		OsMessageBoxQ[coreId][wr].op = op;
		OsMessageBoxQ[coreId][wr].arg1 = arg1;
		OsMessageBoxQ[coreId][wr].arg2 = arg2;
		OsMessageBoxQ[coreId][wr].arg3 = arg3;
		OsMessageBoxQ[coreId][wr].opFinished = false;
		OsMessageBoxQ[coreId][wr].result = E_NOT_OK;

		Os_ArchReleaseSpinlock(&qLock[coreId]);

		return Os_ArchNotifyCore(coreId);
	}

	return E_OK;
}
