/**
 * @file file_handler.c
 * @author Giuseppe Sensolini
 * @brief file handler to store application settings
 * @date 2019-03-10
 * 
 * @copyright Copyright (c) 2019
 * 
 */


#include "file_handler.h"

/**
 * @brief get float array from one text file row
 * 
 * @param file_name name of the .txt file
 * @param buf where will be stored the array
 * @param which row to read. Start from one.
 */
void arrayFromTextFile(const char* file_name, float* buf, int row){
	int ret __attribute__((unused));
	unsigned short counter = row;	
	char str[256] = {};
	unsigned short start=0, end=0, k=0, i=0;
	

	FILE* file = fopen(file_name, "r");
	while( counter>0 ) {
		ret = fscanf(file, "%s" , str);
		counter--;
	}

	while(1){
		if(str[i]=='[' && counter==0){
			i++;
			break;
		}
		i++;
	}

	for( ; str[i-1]!=']'; i++){
		start=i;
		while( !(str[i]==',' || str[i]==']') ){	
			i++;
		}
		end=i;
		char* c_buf = (char*)malloc(sizeof(char)*(end-start));
		memcpy(c_buf, str+start, end-start );
		buf[k] = (float)atof(c_buf);
		free(c_buf);
		k++;
	}
	return;
}


/**
 * @brief store string in a .txt file
 * 
 * @param file_name name of the .txt file
 * @param buf string to write
 */
void stringToFile(const char* file_name, char* buf){
	FILE* file = fopen(file_name, "w");
	if(file == NULL){
		printf("cannot create/open %s file!\n", file_name);
		exit(EXIT_FAILURE);             
	}
	fprintf(file, "%s", buf);
	fclose(file);
	return;
}
