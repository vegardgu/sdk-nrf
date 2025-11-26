/*
 * Copyright (c) 2019 - 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file multithreading_lock.h
 *
 * @brief APIs for ensuring MPSL and BLE controller threadsafe operation.
 */

#ifndef MULTITHREADING_LOCK_H__
#define MULTITHREADING_LOCK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>

/** Macro for acquiring a lock */
#define MULTITHREADING_LOCK_ACQUIRE() \
	multithreading_lock_acquire(K_FOREVER)

/** Macro for acquiring a lock without waiting. */
#define MULTITHREADING_LOCK_ACQUIRE_NO_WAIT() \
	multithreading_lock_acquire(K_NO_WAIT)

/** Macro for acquiring a lock while waiting forever. */
#define MULTITHREADING_LOCK_ACQUIRE_FOREVER_WAIT() \
	multithreading_lock_acquire(K_FOREVER)

/** Macro for releasing a lock */
#define MULTITHREADING_LOCK_RELEASE() multithreading_lock_release()

/** Macro for acquiring a lock and calling a SoftDevice HCI command. */
#define MULTITHREADING_LOCK_SDC_HCI_CMD(sdc_hci_cmd)		\
	({														\
		int _sdc_lock_err = MULTITHREADING_LOCK_ACQUIRE();	\
		if (!_sdc_lock_err) {								\
			int _sdc_call_err = (int)(sdc_hci_cmd);			\
			MULTITHREADING_LOCK_RELEASE();					\
			_sdc_lock_err = _sdc_call_err;					\
		}													\
		_sdc_lock_err;										\
	})

/** @brief Try to take the lock with the specified blocking behavior.
 *
 * This API call will be blocked for the time specified by @p timeout and then
 * return error code.
 *
 * @param[in] timeout     Timeout value for the locking API.
 *
 * @retval 0              Success
 * @retval -EBUSY         Returned without waiting.
 * @retval -EAGAIN        Waiting period timed out.
 */
int multithreading_lock_acquire(k_timeout_t timeout);

/** @brief Unlock the lock.
 *
 * @note This API is must be called only after lock is obtained.
 */
void multithreading_lock_release(void);

#ifdef __cplusplus
}
#endif

#endif /* MULTITHREADING_LOCK_H__ */
