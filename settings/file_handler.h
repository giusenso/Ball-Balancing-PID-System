/**
 * @file file_handler.h
 * @author Giuseppe Sensolini
 * @brief file handler to store application settings
 * @date 2019-03-10
 * 
 * @copyright Copyright (c) 2019
 * 
 */


#ifndef FILE_HANDLER_H
#define FILE_HANDLER_H

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

void arrayFromTextFile(const char* file_name, float* buf, int row);
void stringToFile(const char* file_name, char* buf);

#endif
