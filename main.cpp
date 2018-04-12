/****************************************************************************/
#include <sched.h>
#include "ecat.h"
#include "plc.h"
#include "ladder.h"

/****************************************************************************/

void my_cyclic(void)
{
    int cycle_counter = 0;
    unsigned int blink = 0;

    // set first wake time in a few cycles
    wakeup_time = system_time_ns() + 10 * cycle_ns;

    while (run) {
        // wait for next period (using adjustable system time)
        wait_period();

        cycle_counter++;

        if (!run) {
            break;
        }

        // receive EtherCAT
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        rt_check_domain_state();

        if (!(cycle_counter % 1000)) {
            rt_check_master_state();
        }

        if (!(cycle_counter % 200)) {
            blink = !blink;
        }

        EC_WRITE_U8(domain1_pd + off_dig_out0, blink ? 0x0A : 0x05);
	if(toggle==0) EC_WRITE_U16(domain1_pd + off_control_out1, 0xe000);
	else          EC_WRITE_U16(domain1_pd + off_control_out1, 0xe400);
	if(toggle==0) EC_WRITE_U16(domain1_pd + off_control_out2+6, 0xe000);
	else          EC_WRITE_U16(domain1_pd + off_control_out2+6, 0xe400);
        toggle = !toggle;
        if (!(cycle_counter % 2200)) { tgn = !tgn;}/*
        if (tgn==0) {
        	EC_WRITE_U32(domain1_pd + off_velocity_out1,   0x10ff00);
        	EC_WRITE_U32(domain1_pd + off_velocity_out2+6, 0xffef00ff);	}
	else{
        	EC_WRITE_U32(domain1_pd + off_velocity_out1,   0xffef00ff);
        	EC_WRITE_U32(domain1_pd + off_velocity_out2+6, 0x10ff00);	}
        */
	if (*int_memory[0]==0) {
        	EC_WRITE_U32(domain1_pd + off_velocity_out1,   0x10ff00);
        	EC_WRITE_U32(domain1_pd + off_velocity_out2+6, 0xffef00ff);	}
	else{
        	EC_WRITE_U32(domain1_pd + off_velocity_out1,   0xffef00ff);
        	EC_WRITE_U32(domain1_pd + off_velocity_out2+6, 0x10ff00);	}

        // queue process data
        ecrt_domain_queue(domain1);

        // sync distributed clock just before master_send to set
        // most accurate master clock time
        sync_distributed_clocks();

        // send EtherCAT data
        ecrt_master_send(master);

        // update the master clock
        // Note: called after ecrt_master_send() to reduce time
        // jitter in the sync_distributed_clocks() call
        update_master_clock();
    }
}

/****************************************************************************
 * Main function
 ***************************************************************************/

int main(int argc, char *argv[])
{
    init_plc(argc, argv);
    init_ecat();
    /* Set the initial master time and select a slave to use as the DC
     * reference clock, otherwise pass NULL to auto select the first capable
     * slave. Note: This can be used whether the master or the ref slave will
     * be used as the systems master DC clock.
     */
    dc_start_time_ns = system_time_ns();
    dc_time_ns = dc_start_time_ns;

    /* Attention: The initial application time is also used for phase
     * calculation for the SYNC0/1 interrupts. Please be sure to call it at
     * the correct phase to the realtime cycle.
     */
    ecrt_master_application_time(master, dc_start_time_ns);

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        fprintf(stderr, "Failed to get domain data pointer.\n");
        return -1;
    }

    /* Create cyclic RT-thread */
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        puts("ERROR IN SETTING THE SCHEDULER");
        perror("errno");
        return -1;
    }
/*
    pid_t pid = getpid();
    if (setpriority(PRIO_PROCESS, pid, -19))
        fprintf(stderr, "Warning: Failed to set priority: %s\n",
                strerror(errno));
*/
    printf("Starting cyclic function.\n");
    my_cyclic();

    printf("End of Program\n");
    ecrt_release_master(master);

    return 0;
}

/****************************************************************************/
