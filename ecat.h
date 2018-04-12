/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 ****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <signal.h>
#include <time.h>
#include <sys/mman.h>
#include <errno.h>
#include <sys/resource.h>

#include "ecrt.h"
#include "config_idns.h"
/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)

#define CLOCK_TO_USE CLOCK_REALTIME

typedef long long RTIME;
static unsigned int cycle_ns = 2000000; /* 1 ms */

static int run = 1, toggle=0, tgn=0;

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static uint8_t *domain1_pd = NULL;

static ec_slave_config_t *sc_dig_out_01 = NULL;

/****************************************************************************/

// EtherCAT distributed clock variables

#define DC_FILTER_CNT          1024
#define SYNC_MASTER_TO_REF        1

static uint64_t dc_start_time_ns = 0LL;
static uint64_t dc_time_ns = 0;
#if SYNC_MASTER_TO_REF
static uint8_t  dc_started = 0;
static int32_t  dc_diff_ns = 0;
static int32_t  prev_dc_diff_ns = 0;
static int64_t  dc_diff_total_ns = 0LL;
static int64_t  dc_delta_total_ns = 0LL;
static int      dc_filter_idx = 0;
static int64_t  dc_adjust_ns;
#endif
static int64_t  system_time_base = 0LL;
static uint64_t wakeup_time = 0LL;

//int64_t sb[20]; uint32_t rt[20]; int loop = 0, ctr, qnt=0;
/****************************************************************************/

// process data


#define BusCoupler01_Pos  0, 0
//#define DigOutSlave01_Pos 0, 1
#define DigInSlavePos  0, 1
#define DigOutSlavePos 2, 0
#define EndCouplerPos  2, 1
#define DriveAX5206Pos 2, 2

#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL2004 0x00000002, 0x07d43052
#define Beckhoff_EL1004 0x00000002, 0x03ec3052
#define Beckhoff_EK1110 0x00000002, 0x04562c52
#define Beckhoff_AX5206 0x00000002, 0x14566012

// offsets for PDO entries
static unsigned int off_dig_out0 = 0;
static unsigned int off_control_out1 = 0;
static unsigned int off_velocity_out1 = 0;
static unsigned int off_control_out2 = 0;
static unsigned int off_velocity_out2 = 0;
static unsigned int off_status_in1 = 0;
static unsigned int off_position_in1 = 0;
static unsigned int off_status_in2 = 0;
static unsigned int off_position_in2 = 0;

// process data

const static ec_pdo_entry_reg_t domain1_regs[] = {
   {DigOutSlavePos, Beckhoff_EL2004, 0x3001, 0x01, &off_dig_out0, NULL},
   {DriveAX5206Pos, Beckhoff_AX5206, 0x0086, 0x00, &off_control_out1, NULL},
   {DriveAX5206Pos, Beckhoff_AX5206, 0x0024, 0x00, &off_velocity_out1, NULL},
   {DriveAX5206Pos, Beckhoff_AX5206, 0x0086, 0x00, &off_control_out2, NULL},
   {DriveAX5206Pos, Beckhoff_AX5206, 0x0024, 0x00, &off_velocity_out2, NULL},
   {DriveAX5206Pos, Beckhoff_AX5206, 0x0087, 0x00, &off_status_in1, NULL},
   {DriveAX5206Pos, Beckhoff_AX5206, 0x0033, 0x00, &off_position_in1, NULL},
   {DriveAX5206Pos, Beckhoff_AX5206, 0x0087, 0x00, &off_status_in2, NULL},
   {DriveAX5206Pos, Beckhoff_AX5206, 0x0033, 0x00, &off_position_in2, NULL},
   {}
};

/****************************************************************************/
// Drive Ax5206 --------------------------

static ec_pdo_entry_info_t ax5206_pdo_entries[] = {
    {134, 0,  16}, // master control word          (MDT1)
    {36,  0,  32}, // velocity command value
    {134, 0,  16}, // master control word          (MDT2)
    {36,  0,  32}, // velocity command value
    {135, 0,  16}, // drive status word            (AT1)
    {51,  0,  32}, // position feedback value
    {135, 0,  16}, // drive status word            (AT2)
    {51,  0,  32}, // position feedback value
};

static ec_pdo_info_t ax5206_pdos[] = {
    {0x0018, 2, ax5206_pdo_entries},
    {0x1018, 2, ax5206_pdo_entries + 2},
    {0x0010, 2, ax5206_pdo_entries + 4},
    {0x1010, 2, ax5206_pdo_entries + 6}
};

static ec_sync_info_t ax5206_syncs[] = {
    {2, EC_DIR_OUTPUT,2, ax5206_pdos},
    {3, EC_DIR_INPUT, 2, ax5206_pdos +2},
    {0xff}
};

// Digital In ------------------------

static ec_pdo_entry_info_t el1004_channels[] = {
    {0x3101, 1, 1}, // Value 1
    {0x3101, 2, 1}, // Value 2
    {0x3101, 3, 1}, // Value 3
    {0x3101, 4, 1}  // Value 4
};

static ec_pdo_info_t el1004_pdos[] = {
    {0x1a00, 1, &el1004_channels[0]},
    {0x1a01, 1, &el1004_channels[1]},
    {0x1a02, 1, &el1004_channels[2]},
    {0x1a03, 1, &el1004_channels[3]}
};

static ec_sync_info_t el1004_syncs[] = {
	{0, EC_DIR_OUTPUT},
    {1, EC_DIR_INPUT, 4, el1004_pdos},
    {0xff}
};

// Digital out ------------------------

static ec_pdo_entry_info_t el2004_channels[] = {
    {0x3001, 1, 1}, // Value 1
    {0x3001, 2, 1}, // Value 2
    {0x3001, 3, 1}, // Value 3
    {0x3001, 4, 1}  // Value 4
};

static ec_pdo_info_t el2004_pdos[] = {
    {0x1600, 1, &el2004_channels[0]},
    {0x1601, 1, &el2004_channels[1]},
    {0x1602, 1, &el2004_channels[2]},
    {0x1603, 1, &el2004_channels[3]}
};

static ec_sync_info_t el2004_syncs[] = {
    {0, EC_DIR_OUTPUT, 4, el2004_pdos},
    {1, EC_DIR_INPUT},
    {0xff}
};


/*****************************************************************************
 * Realtime task
 ****************************************************************************/
RTIME time2ns(struct timespec* tsp)
{
	RTIME ret;
        ret =  tsp->tv_sec*(RTIME)1000000000L + tsp->tv_nsec;
	return ret;
}
struct timespec ns2spec(RTIME time)
{
	struct timespec ret;
        ret.tv_sec  = time/(RTIME)1000000000L;
        ret.tv_nsec = time%(RTIME)1000000000L;
	return ret;
}
/** Get the time in ns for the current cpu, adjusted by system_time_base.
 *
 * \attention Rather than calling rt_get_time_ns() directly, all application
 * time calls should use this method instead.
 *
 * \ret The time in ns.
 */
uint64_t system_time_ns(void)
{
    struct timespec tmpt;
    clock_gettime(CLOCK_TO_USE, &tmpt);
    RTIME time = time2ns(&tmpt);

    if (system_time_base > time) {
        printf("%s() error: system_time_base greater than system time (system_time_base: %lld, time: %llu  nt:   )\n", __func__, system_time_base, time);
	exit(-1);
        return time;
    }
    else
        return time - system_time_base;
}

RTIME system2new( uint64_t time )
{
    RTIME ret;
    if ((system_time_base < 0) && ((uint64_t) (-system_time_base) > time))
    {
        printf("%s() error: system_time_base less than system time (system_time_base: %lld, time: %llu nt:  )\n", __func__, system_time_base, time);
        ret = time;
	exit(-1);
    }
    else
	ret = time + system_time_base;

    return ret;
}

/*****************************************************************************/

/** Synchronise the distributed clocks
 */
void sync_distributed_clocks(void)
{
#if SYNC_MASTER_TO_REF
    uint32_t ref_time = 0;
    uint64_t prev_app_time = dc_time_ns;
#endif

    dc_time_ns = system_time_ns();

    // set master time in nano-seconds
    ecrt_master_application_time(master, dc_time_ns);

#if SYNC_MASTER_TO_REF
    // get reference clock time to synchronize master cycle
    ecrt_master_reference_clock_time(master, &ref_time);
    dc_diff_ns = (uint32_t) prev_app_time - ref_time;
#else
    // sync reference clock to master
    ecrt_master_sync_reference_clock(master);
#endif

    // call to sync slaves to ref slave
    ecrt_master_sync_slave_clocks(master);
}

/*****************************************************************************/

/** Return the sign of a number
 *
 * ie -1 for -ve value, 0 for 0, +1 for +ve value
 *
 * \retval the sign of the value
 */
#define sign(val) \
    ({ typeof (val) _val = (val); \
    ((_val > 0) - (_val < 0)); })

/*****************************************************************************/

/** Update the master time based on ref slaves time diff
 *
 * called after the ethercat frame is sent to avoid time jitter in
 * sync_distributed_clocks()
 */
void update_master_clock(void)
{
#if SYNC_MASTER_TO_REF
    // calc drift (via un-normalised time diff)
    int32_t delta = dc_diff_ns - prev_dc_diff_ns;
    prev_dc_diff_ns = dc_diff_ns;

    // normalise the time diff
    dc_diff_ns =
        ((dc_diff_ns + (cycle_ns / 2)) % cycle_ns) - (cycle_ns / 2);

    // only update if primary master
    if (dc_started) {

        // add to totals
        dc_diff_total_ns += dc_diff_ns;
        dc_delta_total_ns += delta;
        dc_filter_idx++;

        if (dc_filter_idx >= DC_FILTER_CNT) {
            // add rounded delta average
            dc_adjust_ns +=
                ((dc_delta_total_ns + (DC_FILTER_CNT / 2)) / DC_FILTER_CNT);

            // and add adjustment for general diff (to pull in drift)
            dc_adjust_ns += sign(dc_diff_total_ns / DC_FILTER_CNT);

            // limit crazy numbers (0.1% of std cycle time)
            if (dc_adjust_ns < -1000) {
                dc_adjust_ns = -1000;
            }
            if (dc_adjust_ns > 1000) {
                dc_adjust_ns =  1000;
            }

            // reset
            dc_diff_total_ns = 0LL;
            dc_delta_total_ns = 0LL;
            dc_filter_idx = 0;
        }

        // add cycles adjustment to time base (including a spot adjustment)
        system_time_base += dc_adjust_ns + sign(dc_diff_ns);
    }
    else {
        dc_started = (dc_diff_ns != 0);

        if (dc_started) {
            // output first diff
          //  printf("First master diff: %d.\n", dc_diff_ns);

            // record the time of this initial cycle
            dc_start_time_ns = dc_time_ns;
        }
    }
#endif
}

/****************************************************************************/

void rt_check_domain_state(void)
{
    ec_domain_state_t ds = {};

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
//        printf("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain1_state.wc_state) {
  //      printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/****************************************************************************/

void rt_check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/****************************************************************************/

/** Wait for the next period
 */
void wait_period(void)
{
    //while (1)
    {
        RTIME wakeup_count = system2new(wakeup_time);
	struct timespec tmpt, tmpt0;
        clock_gettime(CLOCK_TO_USE, &tmpt);
	RTIME current_time = time2ns(&tmpt);

        if ( (wakeup_count < current_time) || (wakeup_count > current_time + (50 * cycle_ns))) {
            printf("%s(): unexpected wake time:  %lld  %lld !\n", __func__, wakeup_count, current_time );
	    exit(-1);
        }
        tmpt0 = ns2spec(wakeup_count);
        int res = clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &tmpt0, NULL);
        switch(res){
	    case 0: //good
		break;
            default:
                printf("nano_sleep(): errors!\n");
	        printf("current-time: %lu  %lu   \n", tmpt.tv_sec, tmpt.tv_nsec );
	        printf("wkup-cnt    : %lu  %lu   \n", tmpt0.tv_sec, tmpt0.tv_nsec );
	        printf("current-time: %llu  wkup-count: %llu  wkup-time:  %llu \n", current_time, wakeup_count, wakeup_time );
		exit(-1);
                break;
        }
        // done if we got to here
     //   break;
    }

    // calc next wake time (in sys time)
    wakeup_time += cycle_ns;
}

/****************************************************************************
 * Signal handler
 ***************************************************************************/

void signal_handler(int sig)
{
    run = 0;
}

/****************************************************************************
 * Main function
 ***************************************************************************/

int init_ecat(void)
{
    ec_slave_config_t *sc_ek1100, *sc, *sc_ax;
    int ret;

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		perror("mlockall failed");
		return -1;
    }

    printf("Requesting master...\n");
    master = ecrt_request_master(0);
    if (!master) {
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        return -1;
    }

    printf("Creating slave configurations...\n");

    // Create configuration for bus coupler
    sc_ek1100 =
        ecrt_master_slave_config(master, BusCoupler01_Pos, Beckhoff_EK1100);
    if (!sc_ek1100) {
        return -1;
    }

    printf("............b...\n");
    if (!(sc = ecrt_master_slave_config(
                    master, DigInSlavePos, Beckhoff_EL1004))) {
                    //master, DigOutSlavePos, Beckhoff_EL2032))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc, EC_END, el1004_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    sc_dig_out_01 =
        ecrt_master_slave_config(master, DigOutSlavePos, Beckhoff_EL2004);
    if (!sc_dig_out_01) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_dig_out_01, EC_END, el2004_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    sc = ecrt_master_slave_config(master, EndCouplerPos, Beckhoff_EK1110);
    if (!sc) return -1;

    if (!(sc_ax = ecrt_master_slave_config(
                    master, DriveAX5206Pos, Beckhoff_AX5206))) {
        fprintf(stderr, "Failed to get slave AX5206 configuration.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_ax, EC_END, ax5206_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    config_idns(sc_ax);
    ecrt_slave_config_dc(sc_ax, 0x0730, 0x0003d090, 0, 0x001ab3f0, 0);

    // Create configuration for Domain
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    ret = ecrt_master_select_reference_clock(master, sc_ek1100);
    if (ret < 0) {
        fprintf(stderr, "Failed to select reference clock: %s\n", strerror(-ret));
        return ret;
    }

    return 0;
}

/****************************************************************************/
