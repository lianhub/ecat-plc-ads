/****************************************************************************/
#include <sched.h>
#include "ecat.h"
#include "plc.h"
#include "ladder.h"

uint32_t s1_32=1, s2_32=2, m1_32=3, m2_32=4, cnt=2;
uint16_t s1_16=1, s2_16=2, m1_16=3, m2_16=4;
uint8_t  v8, bnt=0, dnt=0;
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
if (*int_memory[0]==0) {
        ecrt_domain_process(domain2);
        ecrt_domain_process(domain3);
        ecrt_domain_process(domain4);
      }

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
	if(toggle==0) EC_WRITE_U16(domain1_pd + off_control_out2+6+4, 0xe000);
	else          EC_WRITE_U16(domain1_pd + off_control_out2+6+4, 0xe400);
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
        	EC_WRITE_U32(domain1_pd + off_velocity_out2+6+8, 0xffef00ff);	}
	else{
        	EC_WRITE_U32(domain1_pd + off_velocity_out1,   0xffef00ff);
        	EC_WRITE_U32(domain1_pd + off_velocity_out2+6+8, 0x10ff00);	}

          if (*int_memory[0]==0) {
            s1_32 = EC_READ_U32(domain3_pd + off_el1904_in);
            s1_16 = EC_READ_U16(domain3_pd + off_el1904_in+4);
            s2_32 = EC_READ_U32(domain3_pd + off_el2904_in);
            s2_16 = EC_READ_U16(domain3_pd + off_el2904_in+4);
            v8    = EC_READ_U8 (domain3_pd + off_el1004_in);
            m1_32 = EC_READ_U32(domain4_pd + off_el6900_in1);
            m1_16 = EC_READ_U16(domain4_pd + off_el6900_in1+4);
            m2_32 = EC_READ_U32(domain4_pd + off_el6900_in2);
            m2_16 = EC_READ_U16(domain4_pd + off_el6900_in2+4);
            ///////////////////////////////////////////////////////////////
            EC_WRITE_U32(domain2_pd + off_el1904_out,   m1_32);
            EC_WRITE_U16(domain2_pd + off_el1904_out+4, m1_16);
            EC_WRITE_U32(domain2_pd + off_el2904_out1,   m2_32);
            EC_WRITE_U16(domain2_pd + off_el2904_out1+4, m2_16);
            EC_WRITE_U32(domain4_pd + off_el6900_out1,   s1_32);
            EC_WRITE_U16(domain4_pd + off_el6900_out1+4, s1_16);
            EC_WRITE_U32(domain4_pd + off_el6900_out2,   s2_32);
            EC_WRITE_U16(domain4_pd + off_el6900_out2+4, s2_16);
            if(cnt==2 && bnt<30)
            {EC_WRITE_U8 (domain4_pd + off_el6900_out2+6, 0x1e); cnt=0;bnt++;}
            else
            {  cnt++;
               if(cnt>500 && dnt<3)
               {EC_WRITE_U8 (domain4_pd + off_el6900_out2+6, 0x17); dnt++;}
               else
               EC_WRITE_U8 (domain4_pd + off_el6900_out2+6, 0x16);
            }
          }


        // queue process data
        ecrt_domain_queue(domain1);
if (*int_memory[0]==0) {
        ecrt_domain_queue(domain2);
        ecrt_domain_queue(domain3);
        ecrt_domain_queue(domain4);
      }

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

    if (!(domain1_pd = ecrt_domain_data(domain1)))
    { fprintf(stderr, "Failed to get domain data pointer.\n");        return -1;    }
    if (!(domain2_pd = ecrt_domain_data(domain2)))
    { fprintf(stderr, "Failed to get domain data pointer.\n");        return -1;    }
    if (!(domain3_pd = ecrt_domain_data(domain3)))
    { fprintf(stderr, "Failed to get domain data pointer.\n");        return -1;    }
    if (!(domain4_pd = ecrt_domain_data(domain4)))
    { fprintf(stderr, "Failed to get domain data pointer.\n");        return -1;    }

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
