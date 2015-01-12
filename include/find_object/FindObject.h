/*
Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef FINDOBJECT_H_
#define FINDOBJECT_H_

#include "find_object/FindObjectExp.h" // DLL export/import defines

#include "find_object/DetectionInfo.h"

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QMap>
#include <QtCore/QMultiMap>
#include <QtCore/QPair>
#include <QtCore/QVector>
#include <QtGui/QTransform>
#include <QtCore/QRect>
#include <opencv2/opencv.hpp>
#include <vector>

namespace find_object {

class ObjSignature;
class Vocabulary;
class KeypointDetector;
class DescriptorExtractor;

class FINDOBJECT_EXP FindObject : public QObject
{
	Q_OBJECT;
public:
	static void affineSkew(float tilt,
				float phi,
				const cv::Mat & image,
				cv::Mat & skewImage,
				cv::Mat & skewMask,
				cv::Mat & Ai);

public:
	FindObject(QObject * parent = 0);
	virtual ~FindObject();

	bool loadSession(const QString & path);
	bool saveSession(const QString & path);
	bool isSessionModified() const {return sessionModified_;}

	int loadObjects(const QString & dirPath); // call updateObjects()
	const ObjSignature * addObject(const QString & filePath);
	const ObjSignature * addObject(const cv::Mat & image, int id=0, const QString & filename = QString());
	bool addObject(ObjSignature * obj); // take ownership when true is returned
	void removeObject(int id);
	void removeAllObjects();

	bool detect(const cv::Mat & image, find_object::DetectionInfo & info);

	void updateDetectorExtractor();
	void updateObjects(const QList<int> & ids = QList<int>());
	void updateVocabulary();

	const QMap<int, ObjSignature*> & objects() const {return objects_;}
	const Vocabulary * vocabulary() const {return vocabulary_;}

public Q_SLOTS:
	void addObjectAndUpdate(const cv::Mat & image, int id=0, const QString & filename = QString());
	void removeObjectAndUpdate(int id);
	void detect(const cv::Mat & image); // emit objectsFound()

Q_SIGNALS:
	void objectsFound(const find_object::DetectionInfo &);

private:
	void clearVocabulary();

private:
	QMap<int, ObjSignature*> objects_;
	Vocabulary * vocabulary_;
	QMap<int, cv::Mat> objectsDescriptors_;
	QMap<int, int> dataRange_; // <last id of object's descriptor, id>
	KeypointDetector * detector_;
	DescriptorExtractor * extractor_;
	bool sessionModified_;
};

} // namespace find_object

#endif /* FINDOBJECT_H_ */
