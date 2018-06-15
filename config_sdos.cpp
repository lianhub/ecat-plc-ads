#include <stdio.h>
#include <string.h>
#include "ecrt.h"

struct sdo_set
{
	uint16_t sdo_index;
	uint8_t  sdo_subindex;
	size_t   target_size;
	char     str[300];
	//uint8_t *target;
};

struct sdo_set sdos[10]={
{0x0000fb00, 0x00000001, 0x00000040, "0100000041646d696e6973747261746f7200000003a7cea2f770b4f7c567df06d9d7af34108a1500000000000000000000000000000000000000000000000000"},
{0x00002000, 0x00000000, 0x00000040, "000000000000000000000000000000000000000041646d696e6973747261746f7200000003a7cea2f770b4f7c567df06d9d7af34108a150000000a00000014dc"},
{0x00002040, 0x00000001, 0x00000080, "1a0001000d0002001a0002000d0004001a0004000d0008001b0002000d0001001b0004002000010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003d17"},
{0x00002020, 0x00000001, 0x00000040, "00000400000008000101020000010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003002"},
{0x00002010, 0x00000001, 0x00000040, "0502150c0606000000000000000000000000be0001000300020064000800000f000000000000000000000000000000000000000000000000000000000000146d"},
{0x00002010, 0x00000002, 0x00000040, "0502150c06060000000000000000000000002201020002000200640008000600000000000000000000000000000000000000000000000000000000000000d89d"},
{0x00002030, 0x00000001, 0x00000040, "0b060f000000010000000200000010001d0001001d0002001d0004001d0008000000000000000000000000000000000014140000c8000000000000000000a0a0"},
{0x0000fb00, 0x00000001, 0x00000040, "02000000000000000000000000000000000000000000000000000000000000000000000000000000ff0000000000000000000000000000000000000000000000"}
};

int char22int(char in)
{
	if(in >='0' && in <='9') return in-'0';
	if(in >='A' && in <='F') return in-'A'+10;
	if(in >='a' && in <='f') return in-'a'+10;
}

/*****************************************************************************/
int config_sdos(ec_master_t *master)
{
    fprintf(stdout, "start to configure SDOs.\n");
		int j;
		size_t data_size;		char str[300]=" ";		uint8_t* data = new uint8_t[150];

		size_t result_size;	  uint32_t abort_code;
    uint8_t *target = new uint8_t[264];
		ecrt_master_sdo_upload(master, 5, 0xf880, 1, target, 4, &result_size, &abort_code);
    ecrt_master_sdo_upload(master, 5, 0x100a, 0, target, 2, &result_size, &abort_code);
		ecrt_master_sdo_upload(master, 5, 0xf980, 1, target, 2, &result_size, &abort_code);

			 j=0; strcpy(str, sdos[j].str);			 data_size = strlen(str)/2;
			 for(int i=0; i< data_size; i++) data[i] = char22int(str[2*i])*16 + char22int(str[2*i+1]);
		ecrt_master_sdo_download(master, 5, 0xfb00, 1, data, 64, &abort_code);

		ecrt_master_sdo_upload(master, 5, 0xfb00, 3, target, 64, &result_size, &abort_code );

			 j=1; strcpy(str, sdos[j].str);			 data_size = strlen(str)/2;
			 for(int i=0; i< data_size; i++) data[i] = char22int(str[2*i])*16 + char22int(str[2*i+1]);
		ecrt_master_sdo_download(master, 5, 0x2000, 0, data, 64, &abort_code);
    //ecrt_master_sdo_download(master, 5, 0x2040, 1, data, 128, &abort_code);

		//ecrt_master_sdo_upload(master, 5, 0xfb00, 3, target, 64, &result_size, &abort_code );
		/*
		for(j=2; j<7; j++){
			strcpy(str, sdos[j].str);			 data_size = strlen(str)/2;
			for(int i=0; i< data_size; i++) data[i] = char22int(str[2*i])*16 + char22int(str[2*i+1]);
			//ecrt_master_sdo_download(master, 5, 0x2040, 1, data, 128, abort_code);
		//ecrt_master_sdo_download(master, 5, 0x2040, 1, data, 128, abort_code);
		//ecrt_master_sdo_download(master, 5, 0x2020, 1, data, 64, abort_code);
		//ecrt_master_sdo_download(master, 5, 0x2010, 1, data, 64, abort_code);
		//ecrt_master_sdo_download(master, 5, 0x2010, 2, data, 64, abort_code);
		//ecrt_master_sdo_download(master, 5, 0x2030, 1, data, 64, abort_code);
		ecrt_master_sdo_upload(master, 5, 0xfb00, 3, target, 64, result_size, abort_code );
		//ecrt_master_sdo_download(master, 5, 0xfb00, 1, data, 64, abort_code);
		ecrt_master_sdo_upload(master, 5, 0xfb00, 3, target, 64, result_size, abort_code );
		ecrt_master_sdo_upload(master, 5, 0xf880, 1, target, 4, result_size, abort_code );
		*/
/*
		uint16_t sdo_index = 0xf880;
		uint8_t sdo_subindex = 1;
		uint32_t value = 0;
		//ecrt_slave_config_sdo32( sc, sdo_index, sdo_subindex, value);
*/
		return 0;
}
