/*
 * JsonWriter.h
 *
 *  Created on: 2014-08-03
 *      Author: mathieu
 */

#ifndef JSONWRITER_H_
#define JSONWRITER_H_

#include "find_object/DetectionInfo.h"

class JsonWriter
{
public:
	static bool available();
	static void write(const DetectionInfo & info, const QString & path);
};


#endif /* JSONWRITER_H_ */
