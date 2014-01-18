/* *****************************************************************************
 * fat32.c
 * See atmegaclib2.h header for the copyrights...
 * *****************************************************************************
 */
#ifndef F_CPU
#define F_CPU 16000000UL //required by Atmel Studio 6
#endif
#include <avr/io.h>
#include "atmegaclib2.h"
// TODO: define FAT32 error codes!

// TODO: to be rewritten... depends on PCF8583 RTC functions...
uint8_t getDateTime_FAT(void) {
#ifdef ENABLE_PCF8583
	//it should get the time from RTC but for now, returns "error" code
	return 1;
#else
	return 1; // returning "error" code if RTC not available
			  // date and time are 0 - good enough for testing and budget
#endif
}

//***************************************************************************
//Function: to read data from boot sector of SD card, to determine important
//parameters like bytesPerSector, sectorsPerCluster etc.
//Arguments: none
//return: none
//***************************************************************************
uint8_t F32_getBootSectorData(void) {
	struct BS_Structure *bpb; //mapping the buffer onto the structure
	struct MBRinfo_Structure *mbr;
	struct partitionInfo_Structure *partition;
	uint32_t dataSectors;

	unusedSectors = 0;

	SD_readSingleBlock(0);
	bpb = (struct BS_Structure *) SD_buffer;

	if (bpb->jumpBoot[0] != 0xE9 && bpb->jumpBoot[0] != 0xEB)//check if it is boot sector
	{
		mbr = (struct MBRinfo_Structure *) SD_buffer; //if it is not boot sector, it must be MBR

		if (mbr->signature != 0xaa55)
		return 1;//if it is not even MBR then it's not FAT32

		partition = (struct partitionInfo_Structure *) (mbr->partitionData);//first partition
		unusedSectors = partition->firstSector;//the unused sectors, hidden to the FAT

		SD_readSingleBlock(partition->firstSector);//read the bpb sector
		bpb = (struct BS_Structure *) SD_buffer;
		if (bpb->jumpBoot[0] != 0xE9 && bpb->jumpBoot[0] != 0xEB)
		return 1;
	}

	bytesPerSector = bpb->bytesPerSector;
#ifdef ENABLE_SD_CARD_DEBUG
	//serial_puthexU16(bytesPerSector); serial_putchar(' ');
#endif
	sectorPerCluster = bpb->sectorPerCluster;
#ifdef ENABLE_SD_CARD_DEBUG
	//serial_puthexU08(sectorPerCluster); serial_putchar(' ');
#endif
	reservedSectorCount = bpb->reservedSectorCount;
	rootCluster = bpb->rootCluster; // + (sector / sectorPerCluster) +1;
	firstDataSector = bpb->hiddenSectors + reservedSectorCount
	+ (bpb->numberofFATs * bpb->FATsize_F32);

	dataSectors = bpb->totalSectors_F32 - bpb->reservedSectorCount
	- (bpb->numberofFATs * bpb->FATsize_F32);
	totalClusters = dataSectors / sectorPerCluster;
#ifdef ENABLE_SD_CARD_DEBUG
	//serial_puthexU32(totalClusters); serial_putchar(' ');
#endif
	if ((F32_getSetFreeCluster(TOTAL_FREE, GET, 0)) > totalClusters) //check if FSinfo free clusters count is valid
	freeClusterCountUpdated = 0;
	else
	freeClusterCountUpdated = 1;
	return 0;
}

//***************************************************************************
//Function: to calculate first sector address of any given cluster
//Arguments: cluster number for which first sector is to be found
//return: first sector address
//***************************************************************************
uint32_t F32_getFirstSector(uint32_t clusterNumber) {
	return (((clusterNumber - 2) * sectorPerCluster) + firstDataSector);
}

//***************************************************************************
//Function: get cluster entry value from FAT to find out the next cluster in the chain
//or set new cluster entry in FAT
//Arguments: 1. current cluster number, 2. get_set (=GET, if next cluster is to be found or = SET,
//if next cluster is to be set 3. next cluster number, if argument#2 = SET, else 0
//return: next cluster number, if if argument#2 = GET, else 0
//****************************************************************************
uint32_t F32_getSetNextCluster(uint32_t clusterNumber, uint8_t get_set,
		uint32_t clusterEntry) {
	uint16_t FATEntryOffset;
	uint32_t *FATEntryValue;
	uint32_t FATEntrySector;
	uint8_t retry = 0;

//get sector number of the cluster entry in the FAT
	FATEntrySector = unusedSectors + reservedSectorCount
	+ ((clusterNumber * 4) / bytesPerSector);

//get the offset address in that sector number
	FATEntryOffset = (uint16_t) ((clusterNumber * 4) % bytesPerSector);

//read the sector into a buffer
	while (retry < 10) {
		if (!SD_readSingleBlock(FATEntrySector))
		break;
		retry++;
	}

//get the cluster address from the buffer
	FATEntryValue = (uint32_t *) &SD_buffer[FATEntryOffset];

	if (get_set == GET)
	return ((*FATEntryValue) & 0x0fffffff);

	*FATEntryValue = clusterEntry;//for setting new value in cluster entry in FAT

	SD_writeSingleBlock(FATEntrySector);

	return (0);
}

//********************************************************************************************
//Function: to get or set next free cluster or total free clusters in FSinfo sector of SD card
//Arguments: 1.flag:TOTAL_FREE or NEXT_FREE,
//       2.flag: GET or SET
//       3.new FS entry, when argument2 is SET; or 0, when argument2 is GET
//return: next free cluster, if arg1 is NEXT_FREE & arg2 is GET
//        total number of free clusters, if arg1 is TOTAL_FREE & arg2 is GET
//      0xffffffff, if any error or if arg2 is SET
//********************************************************************************************
uint32_t F32_getSetFreeCluster(uint8_t totOrNext, uint8_t get_set,
		uint32_t FSEntry) {

	struct FSInfo_Structure *FS = (struct FSInfo_Structure *) &SD_buffer;

	SD_readSingleBlock(unusedSectors + 1);

	if ((FS->leadSignature != 0x41615252)
			|| (FS->structureSignature != 0x61417272)
			|| (FS->trailSignature != 0xaa550000))
	return 0xffffffff;

	if (get_set == GET) {
		if (totOrNext == TOTAL_FREE)
		return (FS->freeClusterCount);
		else
		// when totOrNext = NEXT_FREE
		return (FS->nextFreeCluster);
	} else {
		if (totOrNext == TOTAL_FREE)
		FS->freeClusterCount = FSEntry;
		else
		// when totOrNext = NEXT_FREE
		FS->nextFreeCluster = FSEntry;

		SD_writeSingleBlock(unusedSectors + 1);//update FSinfo
	}
	return 0xffffffff;
}

//***************************************************************************
//Function: to get DIR/FILE list or a single file address (cluster number) or to delete a specified file
//Arguments: #1 - flag: GET_LIST, GET_FILE or DELETE #2 - pointer to file name (0 if arg#1 is GET_LIST)
//return: first cluster of the file, if flag = GET_FILE
//        print file/dir list of the root directory, if flag = GET_LIST
//      Delete the file mentioned in arg#2, if flag = DELETE
//****************************************************************************
struct dir_Structure* F32_findFiles(uint8_t flag, uint8_t *fileName) {
	uint32_t cluster, sector, firstSector, firstCluster, nextCluster;
	struct dir_Structure *dir;
	uint16_t i;
	uint8_t j;

	cluster = rootCluster; //root cluster

	while (1) {
		firstSector = F32_getFirstSector(cluster);

		for (sector = 0; sector < sectorPerCluster; sector++) {
			SD_readSingleBlock(firstSector + sector);

			for (i = 0; i < bytesPerSector; i += 32) {
				dir = (struct dir_Structure *) &SD_buffer[i];

				if (dir->name[0] == EMPTY) //indicates end of the file list of the directory
				{
#ifdef ENABLE_SD_CARD_DEBUG
					if (flag == DELETE)
					serial_puts_f("\r\nFile does not exist!");
#endif
					return 0;
				}
				if ((dir->name[0] != DELETED)
						&& (dir->attrib != ATTR_LONG_NAME)) {
					if ((flag == GET_FILE) || (flag == DELETE)) {
						for (j = 0; j < 11; j++)
						if (dir->name[j] != fileName[j])
						break;
						if (j == 11) {
							if (flag == GET_FILE) {
								appendFileSector = firstSector + sector;
								appendFileLocation = i;
								appendStartCluster =
								(((uint32_t) dir->firstClusterHI) << 16)
								| dir->firstClusterLO;
								fileSize = dir->fileSize;
								return (dir);
							} else //when flag = DELETE
							{
#ifdef ENABLE_SD_CARD_DEBUG
								//TX_NEWLINE;
								serial_puts_f("\r\nDeleting..");
								//TX_NEWLINE;
								//TX_NEWLINE;
#endif
								firstCluster = (((uint32_t) dir->firstClusterHI)
										<< 16) | dir->firstClusterLO;

								//mark file as 'deleted' in FAT table
								dir->name[0] = DELETED;
								SD_writeSingleBlock(firstSector + sector);

								F32_freeMemoryUpdate(ADD, dir->fileSize);

								//update next free cluster entry in FSinfo sector
								cluster = F32_getSetFreeCluster(NEXT_FREE, GET,
										0);
								if (firstCluster < cluster)
								F32_getSetFreeCluster(NEXT_FREE, SET,
										firstCluster);

								//mark all the clusters allocated to the file as 'free'
								while (1) {
									nextCluster = F32_getSetNextCluster(
											firstCluster, GET, 0);
									F32_getSetNextCluster(firstCluster, SET, 0);
									if (nextCluster > 0x0ffffff6) {
#ifdef ENABLE_SD_CARD_DEBUG
										serial_puts_f("\r\nFile deleted!");
#endif
										return 0;
									}
									firstCluster = nextCluster;
								}
							}
						}
					} else //when flag = GET_LIST
					{
#ifdef ENABLE_SD_CARD_DEBUG
						TX_NEWLINE;
#endif
						for (j = 0; j < 11; j++) {
#ifdef ENABLE_SD_CARD_DEBUG
							if (j == 8)
							serial_putc(' ');
							serial_putc(dir->name[j]);
#endif
						}
#ifdef ENABLE_SD_CARD_DEBUG
						serial_puts("   ");
#endif
						if ((dir->attrib != 0x10) && (dir->attrib != 0x08)) {
#ifdef ENABLE_SD_CARD_DEBUG
							serial_puts_f("FILE");
							serial_puts_f("   ");
#endif
							F32_displayMemory(LOW, dir->fileSize);
						} else {
#ifdef ENABLE_SD_CARD_DEBUG
							serial_putstr(
									(dir->attrib == 0x10) ? (int8_t *)"DIR\0" : (int8_t *)"ROOT\0");
#endif
						}
					}
				}
			}
		}

		cluster = (F32_getSetNextCluster(cluster, GET, 0));

		if (cluster > 0x0ffffff6)
		return 0;
		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			serial_puts_f("\r\nError in getting cluster");
#endif
			return 0;
		}
	}
	return 0;
}

//***************************************************************************
//Function: if flag=READ then to read file from SD card and send contents to UART
//if flag=VERIFY then functions will verify whether a specified file is already existing
//Arguments: flag (READ or VERIFY) and pointer to the file name
//return: 0, if normal operation or flag is READ
//        1, if file is already existing and flag = VERIFY; or if flag=READ and file does not exist
//      2, if file name is incompatible
//***************************************************************************
uint8_t F32_readFile(uint8_t flag, uint8_t *fileName) {
	struct dir_Structure *dir;
	uint32_t cluster, byteCounter = 0, fileSize, firstSector;
	uint16_t k;
	uint8_t j, error;

	//error = F32_convertFileName(fileName); //convert fileName into FAT format
	//if (error)
	//	return 2;

	dir = F32_findFiles(GET_FILE, fileName);//get the file location
	if (dir == 0) {
		if (flag == READ)
		return (1);
		else
		return (0);
	}

	if (flag == VERIFY)
	return (1); //specified file name is already existing

	cluster = (((uint32_t) dir->firstClusterHI) << 16) | dir->firstClusterLO;

	fileSize = dir->fileSize;
#ifdef ENABLE_SD_CARD_DEBUG
	//TX_NEWLINE;
	//TX_NEWLINE;
#endif
	while (1) {
		firstSector = F32_getFirstSector(cluster);

		for (j = 0; j < sectorPerCluster; j++) {
			SD_readSingleBlock(firstSector + j);

			for (k = 0; k < 512; k++) {
#ifdef ENABLE_SD_CARD_DEBUG
				serial_putc(SD_buffer[k]);
#endif
				if ((byteCounter++) >= fileSize)
				return 0;
			}
		}
		cluster = F32_getSetNextCluster(cluster, GET, 0);
		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			serial_puts_f("\r\nError in getting cluster");
#endif
			return 0;
		}
	}
	return 0;
}

//***************************************************************************
//Function: to convert normal short file name into FAT format
//Arguments: pointer to the file name
//return: 0, if successful else 1.
//***************************************************************************
/*uint8_t F32_convertFileName(uint8_t *fileName) {
 uint8_t fileNameFAT[11];
 uint8_t i, j, k;


 //i = 0;
 //for (j = 0; j < 12; j++){
 //	if (fileName[j] != '.') {
 //		fileNameFAT[i] = fileName[j];
 //		i = i + 1;
 //	}

 //}

 for (j = 0; j < 12; j++)
 if (fileName[j] == '.')
 break;

 if (j > 8) {
 #ifdef ENABLE_SD_CARD_DEBUG
 serial_putstr_f((const int8_t *)"\r\nInvalid fileName..\0");
 #endif
 return 1;

 }

 for (k = 0; k < j; k++) //setting file name
 fileNameFAT[k] = fileName[k];

 for (k = j; k <= 7; k++) //filling file name trail with blanks
 fileNameFAT[k] = ' ';

 j++;
 for (k = 8; k < 11; k++) //setting file extension
 {
 if (fileName[j] != 0)
 fileNameFAT[k] = fileName[j++];
 else
 //filling extension trail with blanks
 while (k < 11)
 fileNameFAT[k++] = ' ';
 }

 for (j = 0; j < 11; j++) //converting small letters to caps
 if ((fileNameFAT[j] >= 0x61) && (fileNameFAT[j] <= 0x7a))
 fileNameFAT[j] -= 0x20;

 for (j = 0; j < 11; j++)
 fileName[j] = fileNameFAT[j];

 return 0;
 }*/

//************************************************************************************
//Function: to create a file in FAT32 format in the root directory if given
//      file name does not exist; if the file already exists then append the data
//Arguments: pointer to the file name
//return: none
//************************************************************************************
uint8_t F32_writeFile(uint8_t *fileName, uint8_t *dataString) {
	uint8_t j, k, data = 0, error, fileCreatedFlag = 0, start = 0, appendFile =
	0, sector = 0;
	uint16_t i, firstClusterHigh = 0, firstClusterLow = 0; //value 0 is assigned just to avoid warning in compilation
	struct dir_Structure *dir;
	uint32_t cluster, nextCluster, prevCluster, firstSector, clusterCount,
	extraMemory;

	j = F32_readFile(VERIFY, fileName);

	if (j == 1) {
#ifdef ENABLE_SD_CARD_DEBUG
		serial_puts_f("\r\nFile already exists, appending data..");
#endif
		appendFile = 1;
		cluster = appendStartCluster;
		clusterCount = 0;
		while (1) {
			nextCluster = F32_getSetNextCluster(cluster, GET, 0);
			if (nextCluster == FAT32_EOF)
			break;
			cluster = nextCluster;
			clusterCount++;
		}

		sector = (fileSize - (clusterCount * sectorPerCluster * bytesPerSector))
		/ bytesPerSector; //last sector number of the last cluster of the file
		start = 1;
	} else if (j == 2)
	return 1; //invalid file name

	else {
#ifdef ENABLE_SD_CARD_DEBUG
		//TX_NEWLINE;
		serial_puts_f("\r\nCreating File..");
#endif
		cluster = F32_getSetFreeCluster(NEXT_FREE, GET, 0);
		if (cluster > totalClusters)
		cluster = rootCluster;

		cluster = F32_searchNextFreeCluster(cluster);
		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			//TX_NEWLINE;
			serial_puts_f("\r\nNo free cluster!");
#endif
			return 1;
		}
		F32_getSetNextCluster(cluster, SET, FAT32_EOF); //last cluster of the file, marked EOF

		firstClusterHigh = (uint16_t) ((cluster & 0xffff0000) >> 16);
		firstClusterLow = (uint16_t) (cluster & 0x0000ffff);
		fileSize = 0;
	}

	k = 0;

	while (1) {
		if (start) {
			start = 0;
			SD_startBlock = F32_getFirstSector(cluster) + sector;
			SD_readSingleBlock(SD_startBlock);
			i = fileSize % bytesPerSector;
			j = sector;
		} else {
			SD_startBlock = F32_getFirstSector(cluster);
			i = 0;
			j = 0;
		}

		do {
			data = dataString[k++];
#ifdef ENABLE_SD_CARD_DEBUG
			serial_putchar(data); // was data
#endif
			SD_buffer[i++] = data;
			fileSize++;

			if (i >= 512) //though 'i' will never become greater than 512, it's kept here to avoid
			{ //infinite loop in case it happens to be greater than 512 due to some data corruption
				i = 0;
				error = SD_writeSingleBlock(SD_startBlock);
				j++;
				if (j == sectorPerCluster) {
					j = 0;
					break;
				}
				SD_startBlock++;
			}
		}while ((data != '\n') && (k < MAX_STRING_SIZE)); //stop when newline character is found
		//or when string size limit reached

		if ((data == '\n') || (k >= MAX_STRING_SIZE)) {
			for (; i < 512; i++) //fill the rest of the buffer with 0x00
			SD_buffer[i] = 0x00;
			error = SD_writeSingleBlock(SD_startBlock);

			break;
		}

		prevCluster = cluster;

		cluster = F32_searchNextFreeCluster(prevCluster); //look for a free cluster starting from the current cluster

		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			//TX_NEWLINE;
			serial_puts_f("\r\nNo free cluster!");
#endif
			return 1;
		}

		F32_getSetNextCluster(prevCluster, SET, cluster);
		F32_getSetNextCluster(cluster, SET, FAT32_EOF); //last cluster of the file, marked EOF
	}

	F32_getSetFreeCluster(NEXT_FREE, SET, cluster); //update FSinfo next free cluster entry

	error = getDateTime_FAT();//get current date & time from the RTC
	if (error) {
		dateFAT = 0;
		timeFAT = 0;
	}

	if (appendFile) //executes this loop if file is to be appended
	{
		SD_readSingleBlock(appendFileSector);
		dir = (struct dir_Structure *) &SD_buffer[appendFileLocation];

		dir->lastAccessDate = 0; //date of last access ignored
		dir->writeTime = timeFAT;//setting new time of last write, obtained from RTC
		dir->writeDate = dateFAT;//setting new date of last write, obtained from RTC
		extraMemory = fileSize - dir->fileSize;
		dir->fileSize = fileSize;
		SD_writeSingleBlock(appendFileSector);
		F32_freeMemoryUpdate(REMOVE, extraMemory);//updating free memory count in FSinfo sector;

#ifdef ENABLE_SD_CARD_DEBUG
		//TX_NEWLINE;
		serial_puts_f("\r\nFile appended!");
#endif
		return 0;
	}

//executes following portion when new file is created

	prevCluster = rootCluster;//root cluster

	while (1) {
		firstSector = F32_getFirstSector(prevCluster);

		for (sector = 0; sector < sectorPerCluster; sector++) {
			SD_readSingleBlock(firstSector + sector);

			for (i = 0; i < bytesPerSector; i += 32) {
				dir = (struct dir_Structure *) &SD_buffer[i];

				if (fileCreatedFlag) //to mark last directory entry with 0x00 (empty) mark
				{ //indicating end of the directory file list
				  //dir->name[0] = EMPTY;
				  //SD_writeSingleBlock (firstSector + sector);
					return 0;
				}

				if ((dir->name[0] == EMPTY) || (dir->name[0] == DELETED)) //looking for an empty slot to enter file info
				{
					for (j = 0; j < 11; j++)
					dir->name[j] = fileName[j];
					dir->attrib = ATTR_ARCHIVE; //setting file attribute as 'archive'
					dir->NTreserved = 0;//always set to 0
					dir->timeTenth = 0;//always set to 0
					dir->createTime = timeFAT;//setting time of file creation, obtained from RTC
					dir->createDate = dateFAT;//setting date of file creation, obtained from RTC
					dir->lastAccessDate = 0;//date of last access ignored
					dir->writeTime = timeFAT;//setting new time of last write, obtained from RTC
					dir->writeDate = dateFAT;//setting new date of last write, obtained from RTC
					dir->firstClusterHI = firstClusterHigh;
					dir->firstClusterLO = firstClusterLow;
					dir->fileSize = fileSize;

					SD_writeSingleBlock(firstSector + sector);
					fileCreatedFlag = 1;
#ifdef ENABLE_SD_CARD_DEBUG
					//TX_NEWLINE;
					//TX_NEWLINE;
					serial_puts_f("\r\nFile Created! ");
#endif

					F32_freeMemoryUpdate(REMOVE, fileSize); //updating free memory count in FSinfo sector

				}
			}
		}

		cluster = F32_getSetNextCluster(prevCluster, GET, 0);

		if (cluster > 0x0ffffff6) {
			if (cluster == FAT32_EOF) //this situation will come when total files in root is multiple of (32*sectorPerCluster)
			{
				cluster = F32_searchNextFreeCluster(prevCluster); //find next cluster for root directory entries
				F32_getSetNextCluster(prevCluster, SET, cluster);//link the new cluster of root to the previous cluster
				F32_getSetNextCluster(cluster, SET, FAT32_EOF);//set the new cluster as end of the root directory
			}

			else {
#ifdef ENABLE_SD_CARD_DEBUG
				serial_puts_f("\r\nEnd of Cluster Chain");
#endif
				return 1;
			}
		}
		if (cluster == 0) {
#ifdef ENABLE_SD_CARD_DEBUG
			serial_puts_f("\r\nError in getting cluster");
#endif
			return 1;
		}

		prevCluster = cluster;
	}

	return 0;
}

//***************************************************************************
//Function: to search for the next free cluster in the root directory
//          starting from a specified cluster
//Arguments: Starting cluster
//return: the next free cluster
//****************************************************************
uint32_t F32_searchNextFreeCluster(uint32_t startCluster) {
	uint32_t cluster, *value, sector;
	uint8_t i;

	startCluster -= (startCluster % 128); //to start with the first file in a FAT sector
	for (cluster = startCluster; cluster < totalClusters; cluster += 128) {
		sector = unusedSectors + reservedSectorCount
		+ ((cluster * 4) / bytesPerSector);
		SD_readSingleBlock(sector);
		for (i = 0; i < 128; i++) {
			value = (uint32_t *) &SD_buffer[i * 4];
			if (((*value) & 0x0fffffff) == 0)
			return (cluster + i);
		}
	}

	return 0;
}

//************************************************************
//Function: To convert the uint32_t value of memory into
//          text string and send to UART
//Arguments: 1. uint8_t flag. If flag is HIGH, memory will be displayed in KBytes, else in Bytes.
//       2. uint32_t memory value
//return: none
//************************************************************
void F32_displayMemory(uint8_t flag, uint32_t memory) {
	uint8_t memoryString[] = "              Bytes"; //19 character long string for memory display
	uint8_t i;
	for (i = 12; i > 0; i--)//converting freeMemory into ASCII string
	{
		if (i == 5 || i == 9) {
			memoryString[i - 1] = ',';
			i--;
		}
		memoryString[i - 1] = (memory % 10) | 0x30;
		memory /= 10;
		if (memory == 0)
		break;
	}
	if (flag == HIGH)
	memoryString[13] = 'K';
#if defined(ENABLE_SERIAL) || defined(ENABLE_SERIAL_POLL)
	serial_puts(memoryString);
#endif
}

//********************************************************************
//Function: to delete a specified file from the root directory
//Arguments: pointer to the file name
//return: none
//********************************************************************
void F32_deleteFile(uint8_t *fileName) {
	//uint8_t error;

	//error = F32_convertFileName(fileName);
	//if (error)
	//	return;

	F32_findFiles(DELETE, fileName);
}

//********************************************************************
//Function: update the free memory count in the FSinfo sector.
//      Whenever a file is deleted or created, this function will be called
//      to ADD or REMOVE clusters occupied by the file
//Arguments: #1.flag ADD or REMOVE #2.file size in Bytes
//return: none
//********************************************************************
void F32_freeMemoryUpdate(uint8_t flag, uint32_t size) {
	uint32_t freeClusters;
	//convert file size into number of clusters occupied
	if ((size % 512) == 0)
	size = size / 512;
	else
	size = (size / 512) + 1;
	if ((size % 8) == 0)
	size = size / 8;
	else
	size = (size / 8) + 1;

	if (freeClusterCountUpdated) {
		freeClusters = F32_getSetFreeCluster(TOTAL_FREE, GET, 0);
		if (flag == ADD)
		freeClusters = freeClusters + size;
		else
		//when flag = REMOVE
		freeClusters = freeClusters - size;
		F32_getSetFreeCluster(TOTAL_FREE, SET, freeClusters);
	}
}

//******** END ****** www.dharmanitech.com *****



