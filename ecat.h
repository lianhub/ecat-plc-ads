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

static ec_domain_t *domain2 = NULL;
static ec_domain_t *domain3 = NULL;
static ec_domain_t *domain4 = NULL;
static uint8_t *domain2_pd = NULL;
static uint8_t *domain3_pd = NULL;
static uint8_t *domain4_pd = NULL;
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
#define EK1100Pos 0, 0
#define EL2084Pos 0, 1
#define EL1904Pos 0, 2
#define EL1004Pos 0, 3
#define EL2904Pos 0, 4
#define EL6900Pos 0, 5
#define EL3008Pos 0, 6
#define EK1110Pos 0, 7
#define AX5206Pos 0, 8

#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL2004 0x00000002, 0x07d43052
#define Beckhoff_EL1004 0x00000002, 0x03ec3052
#define Beckhoff_EK1110 0x00000002, 0x04562c52
#define Beckhoff_AX5206 0x00000002, 0x14566012
#define Beckhoff_EL2084 0x00000002, 0x08243052
#define Beckhoff_EL1904 0x00000002, 0x07703052
#define Beckhoff_EL2904 0x00000002, 0x0B583052
#define Beckhoff_EL6900 0x00000002, 0x1af43052
#define Beckhoff_EL3008 0x00000002, 0x0bc03052

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
   {EL2084Pos, Beckhoff_EL2084, 0x7000, 0x01, &off_dig_out0, NULL},
   {AX5206Pos, Beckhoff_AX5206, 0x0086, 0x00, &off_control_out1, NULL},
   {AX5206Pos, Beckhoff_AX5206, 0x0024, 0x00, &off_velocity_out1, NULL},
   {AX5206Pos, Beckhoff_AX5206, 0x0086, 0x00, &off_control_out2, NULL},
   {AX5206Pos, Beckhoff_AX5206, 0x0024, 0x00, &off_velocity_out2, NULL},
   {AX5206Pos, Beckhoff_AX5206, 0x0087, 0x00, &off_status_in1, NULL},
   {AX5206Pos, Beckhoff_AX5206, 0x0033, 0x00, &off_position_in1, NULL},
   {AX5206Pos, Beckhoff_AX5206, 0x0087, 0x00, &off_status_in2, NULL},
   {AX5206Pos, Beckhoff_AX5206, 0x0033, 0x00, &off_position_in2, NULL},
   {}
};
static unsigned int off_el1904_in = 0;
static unsigned int off_el1904_out = 0;
static unsigned int off_el2904_in = 0;
static unsigned int off_el2904_out1 = 0;
static unsigned int off_el2904_out2 = 0;

static unsigned int off_el6900_in1 = 0;
static unsigned int off_el6900_out1 = 0;
static unsigned int off_el6900_in2 = 0;
static unsigned int off_el6900_out2 = 0;
static unsigned int off_el6900_in3 = 0;
static unsigned int off_el6900_out3 = 0;
static unsigned int off_el6900_in4 = 0;
static unsigned int off_el6900_out4 = 0;
static unsigned int off_el6900_in5 = 0;

static unsigned int off_el1004_in = 0;

const static ec_pdo_entry_reg_t domain2_regs[] = {
   {EL1904Pos, Beckhoff_EL1904, 0x7000, 1, &off_el1904_out, NULL}, //6 bytes
   {EL2904Pos, Beckhoff_EL2904, 0x7000, 1, &off_el2904_out1, NULL}, //6 bytes
   {EL2904Pos, Beckhoff_EL2904, 0x7010, 1, &off_el2904_out2, NULL}, //2 bytes
   //{EL1904Pos, Beckhoff_EL1904, 0x6000, 1, &off_el1904_in, NULL}, //6 bytes
   //{EL2904Pos, Beckhoff_EL2904, 0x6000, 1, &off_el2904_in, NULL}, //6 bytes
   {}
};
const static ec_pdo_entry_reg_t domain3_regs[] = {
   //{EL1904Pos, Beckhoff_EL1904, 0x7000, 1, &off_el1904_out, NULL}, //6 bytes
   //{EL2904Pos, Beckhoff_EL2904, 0x7000, 1, &off_el2904_out1, NULL}, //6 bytes
   //{EL2904Pos, Beckhoff_EL2904, 0x7010, 1, &off_el2904_out2, NULL}, //2 bytes
   {EL1904Pos, Beckhoff_EL1904, 0x6000, 1, &off_el1904_in, NULL}, //6 bytes
   {EL2904Pos, Beckhoff_EL2904, 0x6000, 1, &off_el2904_in, NULL}, //6 bytes

   {EL1004Pos, Beckhoff_EL1004, 0x6000, 1, &off_el1004_in, NULL}, //6 bytes
   {}
};

const static ec_pdo_entry_reg_t domain4_regs[] = {
    {EL6900Pos, Beckhoff_EL6900, 0x7000, 1, &off_el6900_out1, NULL}, //6 bytes
    {EL6900Pos, Beckhoff_EL6900, 0x7010, 1, &off_el6900_out2, NULL}, //6 bytes
    {EL6900Pos, Beckhoff_EL6900, 0xF201, 0, &off_el6900_out3, NULL}, //1 bytes
    {EL6900Pos, Beckhoff_EL6900, 0xF200, 1, &off_el6900_out4, NULL}, //2 bytes
    ///////////////////////////////////////////////////////////////////////////
   {EL6900Pos, Beckhoff_EL6900, 0x6000, 1, &off_el6900_in1, NULL}, //6 bytes
   {EL6900Pos, Beckhoff_EL6900, 0x6010, 1, &off_el6900_in2, NULL}, //6 bytes
   {EL6900Pos, Beckhoff_EL6900, 0xF101, 0, &off_el6900_in3, NULL}, //1 bytes
   {EL6900Pos, Beckhoff_EL6900, 0x2110, 1, &off_el6900_in4, NULL}, //4 bytes
   {EL6900Pos, Beckhoff_EL6900, 0xF100, 1, &off_el6900_in5, NULL}, //2 bytes
   {}
};

/****************************************************************************/
// EL3008 ------------------------
static ec_sync_info_t el3008_syncs[] = {
    {0xff}
};

// EL6900 ------------------------
static ec_pdo_entry_info_t el6900_conn1_tx[] = {
    {0x6000, 1, 8}, // FSOE CMD
    {0x6001, 1, 1}, // FSOE input
    {0x6001, 2, 1}, // FSOE input
    {0x6001, 3, 1}, // FSOE input
    {0x6001, 4, 1}, // FSOE input
    {0,      0, 4}, //
    {0x6000, 3, 16}, // FSOE CRC_0
    {0x6000, 2, 16}, // FSOE ConnID
};
static ec_pdo_entry_info_t el6900_conn2_tx[] = {
    {0x6010, 1, 8}, // FSOE CMD
    {0,      0, 8}, //
    {0x6010, 3, 16}, // FSOE CRC_0
    {0x6010, 2, 16}, // FSOE ConnID
};
static ec_pdo_entry_info_t el6900_conn1_rx[] = {
    {0x7000, 1, 8}, // FSOE CMD
    {0,      0, 8}, //
    {0x7000, 3, 16}, // FSOE CRC_0
    {0x7000, 2, 16}, // FSOE ConnID
};
static ec_pdo_entry_info_t el6900_conn2_rx[] = {
    {0x7010, 1, 8}, // FSOE CMD
    {0x7011, 1, 1}, // FSOE output
    {0x7011, 2, 1}, // FSOE output
    {0x7011, 3, 1}, // FSOE output
    {0x7011, 4, 1}, // FSOE output
    {0,      0, 4}, //
    {0x7010, 3, 16}, // FSOE CRC_0
    {0x7010, 2, 16}, // FSOE ConnID
};
static ec_pdo_entry_info_t el6900_out_var[] = {
    {0xF101, 0, 1}, //
    {0xF101, 1, 1}, //
    {0xF101, 2, 1}, //
    {0xF101, 3, 1}, //
    {0,      0, 4}, //
};
static ec_pdo_entry_info_t el6900_in_var[] = {
    {0xF201, 0, 1}, //
    {0xF201, 1, 1}, //
    {0xF201, 2, 1}, //
    {0xF201, 3, 1}, //
    {0xF201, 4, 1}, //
    {0,      0, 3}, //
};
static ec_pdo_entry_info_t el6900_conn_info[] = {
    {0x2110, 1, 16}, //
    {0x2110, 2, 16}, //
};
static ec_pdo_entry_info_t el6900_dev_in[] = {
    {0xF100, 1,   3}, //
    {0,      0,   4}, //
    {0xF100, 8,   1}, //
    {0xF100, 9,   1}, //
    {0xF100, 0xA, 1}, //
    {0,      0,   4}, //
    {0xF100, 0xF, 1}, //
    {0xF100, 0x10,1}, //
};
static ec_pdo_entry_info_t el6900_dev_out[] = {
    {0xF200, 1, 16}, //
};

static ec_pdo_info_t el6900_pdos[] = {
  {0x1A00, 8, el6900_conn1_tx}, //SM-3, IN
  {0x1A01, 4, el6900_conn2_tx}, //SM-3, IN
  {0x1BF0, 5, el6900_out_var},  //SM-3, IN
  {0x1BF9, 2, el6900_conn_info},//SM-3, IN
  {0x1BFF, 8, el6900_dev_in},   //SM-3, IN
  {0x1600, 4, el6900_conn1_rx}, //SM-2, OUT
  {0x1601, 8, el6900_conn2_rx}, //SM-2, OUT
  {0x17F0, 6, el6900_in_var},   //SM-2, OUT
  {0x17FF, 1, el6900_dev_out},  //SM-2, OUT
};

static ec_sync_info_t el6900_syncs[] = {
    {3, EC_DIR_INPUT, 5, el6900_pdos +0},
    {2, EC_DIR_OUTPUT,4, el6900_pdos +5},
    {0xff}
};

// EL2084 ------------------------
static ec_pdo_entry_info_t el2084_channels[] = {
    {0x7000, 1, 1}, // Value 1
    {0x7010, 1, 1}, // Value 2
    {0x7020, 1, 1}, // Value 3
    {0x7030, 1, 1}  // Value 4
};

static ec_pdo_info_t el2084_pdos[] = {
    {0x1600, 1, &el2084_channels[0]},
    {0x1601, 1, &el2084_channels[1]},
    {0x1602, 1, &el2084_channels[2]},
    {0x1603, 1, &el2084_channels[3]}
};

static ec_sync_info_t el2084_syncs[] = {
    {0, EC_DIR_OUTPUT, 4, el2084_pdos},
    {0xff}
};

// Drive EL1904 --------------------------
static ec_pdo_entry_info_t el1904_pdo_entries[] = {
    {0x7000, 1,  8}, // Command
    {0,      0,  8}, // padding
    {0x7000, 3,  16}, // CRC
    {0x7000, 2,  16}, // Connection ID

    {0x6000, 1,  8}, // Command
    {0x6001, 1,  1}, // In1
    {0x6001, 2,  1}, // In2
    {0x6001, 3,  1}, // In3
    {0x6001, 4,  1}, // In4
    {0,      0,  4}, // padding
    {0x6000, 3,  16}, // CRC
    {0x6000, 2,  16}, // Connection ID
};

static ec_pdo_info_t el1904_pdos[] = {
    {0x1600, 4, el1904_pdo_entries},
    {0x1A00, 8, el1904_pdo_entries + 4}
};

static ec_sync_info_t el1904_syncs[] = {
    {2, EC_DIR_OUTPUT,1, el1904_pdos},
    {3, EC_DIR_INPUT, 1, el1904_pdos +1},
    {0xff}
};

// Drive EL2904 --------------------------
static ec_pdo_entry_info_t el2904_pdo_entries[] = {
    {0x7000, 1,  8}, // Command
    {0x7001, 1,  1}, // FSOE out1
    {0x7001, 2,  1}, // FSOE out2
    {0x7001, 3,  1}, // FSOE out3
    {0x7001, 4,  1}, // FSOE out4
    {0,      0,  4}, // padding
    {0x7000, 3,  16}, // CRC
    {0x7000, 2,  16}, // Connection ID

    {0x7010, 1,  1}, // Drive Output 0
    {0x7010, 2,  1}, // Drive Output 1
    {0x7010, 3,  1}, // Drive Output 2
    {0x7010, 4,  1}, // Drive Output 3
    {0,      0,  12}, // padding

    {0x6000, 1,  8}, // Command
    {0,      0,  8}, // padding
    {0x6000, 3,  16}, // CRC
    {0x6000, 2,  16}, // Connection ID
};

static ec_pdo_info_t el2904_pdos[] = {
    {0x1600, 8, el2904_pdo_entries},
    {0x1601, 5, el2904_pdo_entries + 8},
    {0x1A00, 4, el2904_pdo_entries + 13}
};

static ec_sync_info_t el2904_syncs[] = {
    {2, EC_DIR_OUTPUT,2, el2904_pdos},
    {3, EC_DIR_INPUT, 1, el2904_pdos +2},
    {0xff}
};

// Drive Ax5206 --------------------------
static ec_pdo_entry_info_t ax5206_pdo_entries[] = {
    {134, 0,  16}, // master control word          (MDT1)
    {36,  0,  32}, // velocity command value
    {47,  0,  32}, // Position command value
    {134, 0,  16}, // master control word          (MDT2)
    {47,  0,  32}, // Position command value
    {36,  0,  32}, // velocity command value
    {135, 0,  16}, // drive status word            (AT1)
    {51,  0,  32}, // position feedback value
    {381, 0,  32}, // dc bus current
    {380, 0,  16}, // dc bus voltage
    {189, 0,  32}, // following distance
    {84,  0,  16}, // torque feedback value
    {0x81c6, 0,  16}, // effective torque command value 0x1c6=454
    {135, 0,  16}, // drive status word            (AT2)
    {51,  0,  32}, // position feedback value
    {189, 0,  32}, // following distance
    {84,  0,  16}, // torque feedback value
    {0x81c6, 0,  16}, // effective torque command value 0x1c6=454
};

static ec_pdo_info_t ax5206_pdos[] = {
    {0x0018, 3, ax5206_pdo_entries},
    {0x1018, 3, ax5206_pdo_entries + 3},
    {0x0010, 7, ax5206_pdo_entries + 6},
    {0x1010, 5, ax5206_pdo_entries + 13}
};

static ec_sync_info_t ax5206_syncs[] = {
    {2, EC_DIR_OUTPUT,2, ax5206_pdos},
    {3, EC_DIR_INPUT, 2, ax5206_pdos +2},
    {0xff}
};

// EL1004 Digital In ------------------------
static ec_pdo_entry_info_t el1004_channels[] = {
    {0x6000, 1, 1}, //
    {0x6010, 1, 1}, // edm, inVar-5
    {0x6020, 1, 1}, // RestartEstopRemote
    {0x6030, 1, 1}  // TransformerTemp
};

static ec_pdo_info_t el1004_pdos[] = {
    {0x1A00, 1, el1004_channels},
    {0x1A01, 1, el1004_channels + 1},
    {0x1A02, 1, el1004_channels + 2},
    {0x1A03, 1, el1004_channels + 3}
};

static ec_sync_info_t el1004_syncs[] = {
    {0, EC_DIR_INPUT, 4, el1004_pdos},
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

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
		{   perror("mlockall failed");		return -1;    }

    printf("Requesting master...\n");
    master = ecrt_request_master(0);
    if (!master)         return -1;

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)         return -1;
    domain2 = ecrt_master_create_domain(master);
    if (!domain2)         return -1;
    domain3 = ecrt_master_create_domain(master);
    if (!domain3)         return -1;
    domain4 = ecrt_master_create_domain(master);
    if (!domain4)         return -1;

    printf("Creating slave configurations...\n");

    // Create configuration for bus coupler
    // Box 1**********************************************************************
    sc_ek1100 =  ecrt_master_slave_config(master, EK1100Pos, Beckhoff_EK1100);
    if (!sc_ek1100)    return -1;

    printf("............b...\n");
    // Box 2**********************************************************************
    sc = ecrt_master_slave_config(master, EL2084Pos, Beckhoff_EL2084);
    if (!sc)
    {  fprintf(stderr, "Failed to get slave configuration.\n"); return -1;    }
    if (ecrt_slave_config_pdos(sc, EC_END, el2084_syncs))
    {  fprintf(stderr, "Failed to configure PDOs.\n");          return -1;    }
    // Box 3**********************************************************************
    sc = ecrt_master_slave_config(master, EL1904Pos, Beckhoff_EL1904);
    if (!sc)
    {  fprintf(stderr, "Failed to get slave configuration.\n"); return -1;    }
    if (ecrt_slave_config_pdos(sc, EC_END, el1904_syncs))
    {  fprintf(stderr, "Failed to configure PDOs.\n");          return -1;    }
    // Box 4**********************************************************************
    sc = ecrt_master_slave_config(master, EL1004Pos, Beckhoff_EL1004);
    if (!sc)
    {  fprintf(stderr, "Failed to get slave configuration.\n"); return -1;    }
    if (ecrt_slave_config_pdos(sc, EC_END, el1004_syncs))
    {  fprintf(stderr, "Failed to configure PDOs.\n");          return -1;    }
    // Box 5**********************************************************************
    sc = ecrt_master_slave_config(master, EL2904Pos, Beckhoff_EL2904);
    if (!sc)
    {  fprintf(stderr, "Failed to get slave configuration.\n"); return -1;    }
    if (ecrt_slave_config_pdos(sc, EC_END, el2904_syncs))
    {  fprintf(stderr, "Failed to configure PDOs.\n");          return -1;    }
    // Box 6**********************************************************************
    sc = ecrt_master_slave_config(master, EL6900Pos, Beckhoff_EL6900);
    if (!sc)
    {  fprintf(stderr, "Failed to get slave configuration.\n"); return -1;    }
    if (ecrt_slave_config_pdos(sc, EC_END, el6900_syncs))
    {  fprintf(stderr, "Failed to configure PDOs.\n");          return -1;    }
    // Box 7**********************************************************************
    sc = ecrt_master_slave_config(master, EL3008Pos, Beckhoff_EL3008);
    if (!sc)
    {  fprintf(stderr, "Failed to get slave configuration.\n"); return -1;    }
    if (ecrt_slave_config_pdos(sc, EC_END, el3008_syncs))
    {  fprintf(stderr, "Failed to configure PDOs.\n");          return -1;    }
    // Box 8**********************************************************************
    sc = ecrt_master_slave_config(master, EK1110Pos, Beckhoff_EK1110);
    if (!sc) return -1;
    // Box 9**********************************************************************
    sc_ax = ecrt_master_slave_config(master, AX5206Pos, Beckhoff_AX5206);
    if (!sc_ax)
    {   fprintf(stderr, "Failed to get slave AX5206 configuration.\n");   return -1;  }
    if (ecrt_slave_config_pdos(sc_ax, EC_END, ax5206_syncs))
    {   fprintf(stderr, "Failed to configure PDOs.\n");                   return -1;  }

    //*************Config IDNs*********************************************************
    config_idns(sc_ax);
    ecrt_slave_config_dc(sc_ax, 0x0730, 0x0003d090, 0, 0x001ab3f0, 0);

    // Create configuration for Domain
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs))
    {   fprintf(stderr, "PDO entry registration failed!\n");        return -1;    }
    if (ecrt_domain_reg_pdo_entry_list(domain2, domain2_regs))
    {   fprintf(stderr, "PDO entry registration failed!\n");        return -1;    }
    if (ecrt_domain_reg_pdo_entry_list(domain3, domain3_regs))
    {   fprintf(stderr, "PDO entry registration failed!\n");        return -1;    }
    if (ecrt_domain_reg_pdo_entry_list(domain4, domain4_regs))
    {   fprintf(stderr, "PDO entry registration failed!\n");        return -1;    }

    ret = ecrt_master_select_reference_clock(master, sc_ek1100);
    if (ret < 0)
    {   fprintf(stderr, "Failed to select reference clock: %s\n", strerror(-ret));  return ret; }

    return 0;
}

/****************************************************************************/
