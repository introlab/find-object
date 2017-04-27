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

#include "find_object/FindObject.h"
#include "find_object/Settings.h"
#include "find_object/utilite/ULogger.h"
#include "utilite/UConversion.h"

#include "ObjSignature.h"
#include "utilite/UDirectory.h"
#include "Vocabulary.h"

#include <QtCore/QThread>
#include <QtCore/QFileInfo>
#include <QtCore/QStringList>
#include <QtCore/QTime>
#include <QtCore/QDir>
#include <QGraphicsRectItem>
#include <stdio.h>

namespace find_object {

FindObject::FindObject(bool keepImagesInRAM, QObject * parent) :
	QObject(parent),
	vocabulary_(new Vocabulary()),
	detector_(Settings::createKeypointDetector()),
	extractor_(Settings::createDescriptorExtractor()),
	sessionModified_(false),
	keepImagesInRAM_(keepImagesInRAM)
{
	qRegisterMetaType<find_object::DetectionInfo>("find_object::DetectionInfo");
	UASSERT(detector_ != 0 && extractor_ != 0);

	if(Settings::getGeneral_debug())
	{
		ULogger::setPrintWhere(true);
		ULogger::setLevel(ULogger::kDebug);
	}
	else
	{
		ULogger::setPrintWhere(false);
		ULogger::setLevel(ULogger::kInfo);
	}
}

FindObject::~FindObject() {
	delete detector_;
	delete extractor_;
	delete vocabulary_;
	objectsDescriptors_.clear();
}

bool FindObject::loadSession(const QString & path)
{
	if(QFile::exists(path) && !path.isEmpty() && QFileInfo(path).suffix().compare("bin") == 0)
	{
		QFile file(path);
		file.open(QIODevice::ReadOnly);
		QDataStream in(&file);

		ParametersMap parameters;

		// load parameters
		in >> parameters;
		for(QMap<QString, QVariant>::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
		{
			Settings::setParameter(iter.key(), iter.value());
		}

		// save vocabulary
		vocabulary_->load(in);

		// load objects
		while(!in.atEnd())
		{
			ObjSignature * obj = new ObjSignature();
			obj->load(in, !keepImagesInRAM_);
			if(obj->id() >= 0)
			{
				objects_.insert(obj->id(), obj);
			}
			else
			{
				UERROR("Failed to load and object!");
				delete obj;
			}
		}
		file.close();

		if(!Settings::getGeneral_invertedSearch())
		{
			// this will fill objectsDescriptors_ matrix
			updateVocabulary();
		}
		sessionModified_ = false;
		return true;
	}
	else
	{
		UERROR("Invalid session file (should be *.bin): \"%s\"", path.toStdString().c_str());
	}
	return false;
}

bool FindObject::saveSession(const QString & path)
{
	if(!path.isEmpty() && QFileInfo(path).suffix().compare("bin") == 0)
	{
		QFile file(path);
		file.open(QIODevice::WriteOnly);
		QDataStream out(&file);

		// save parameters
		out << Settings::getParameters();

		// save vocabulary
		vocabulary_->save(out);

		// save objects
		for(QMultiMap<int, ObjSignature*>::const_iterator iter=objects_.constBegin(); iter!=objects_.constEnd(); ++iter)
		{
			iter.value()->save(out);
		}

		file.close();
		sessionModified_ = false;
		return true;
	}
	UERROR("Path \"%s\" not valid (should be *.bin)", path.toStdString().c_str());
	return false;
}

bool FindObject::saveVocabulary(const QString & filePath) const
{
	return vocabulary_->save(filePath);
}

bool FindObject::loadVocabulary(const QString & filePath)
{
	if(!Settings::getGeneral_vocabularyFixed() || !Settings::getGeneral_invertedSearch())
	{
		UWARN("Doesn't make sense to load a vocabulary if \"General/vocabularyFixed\" and \"General/invertedSearch\" are not enabled! It will "
			  "be cleared at the time the objects are updated.");
	}
	if(vocabulary_->load(filePath))
	{
		if(objects_.size())
		{
			updateVocabulary();
		}
		return true;
	}
	return false;
}

int FindObject::loadObjects(const QString & dirPath, bool recursive)
{
	QString formats = Settings::getGeneral_imageFormats().remove('*').remove('.');

	QStringList paths;
	paths.append(dirPath);

	QList<int> idsLoaded;
	while(paths.size())
	{
		QString currentDir = paths.front();
		UDirectory dir(currentDir.toStdString(), formats.toStdString());
		if(dir.isValid())
		{
			const std::list<std::string> & names = dir.getFileNames(); // sorted in natural order
			for(std::list<std::string>::const_iterator iter=names.begin(); iter!=names.end(); ++iter)
			{
				const ObjSignature * s = this->addObject((currentDir.toStdString()+dir.separator()+*iter).c_str());
				if(s)
				{
					idsLoaded.push_back(s->id());
				}
			}
		}

		paths.pop_front();

		if(recursive)
		{
			QDir d(currentDir);
			QStringList subDirs = d.entryList(QDir::AllDirs|QDir::NoDotAndDotDot, QDir::Name);
			for(int i=subDirs.size()-1; i>=0; --i)
			{
				paths.prepend(currentDir+QDir::separator()+subDirs[i]);
			}
		}
	}

	if(idsLoaded.size())
	{
		this->updateObjects(idsLoaded);
		this->updateVocabulary(idsLoaded);
	}

	return idsLoaded.size();
}

const ObjSignature * FindObject::addObject(const QString & filePath)
{
	if(!filePath.isNull())
	{
		cv::Mat img = cv::imread(filePath.toStdString().c_str(), cv::IMREAD_GRAYSCALE);
		if(!img.empty())
		{
			int id = 0;
			QFileInfo file(filePath);
			QStringList list = file.fileName().split('.');
			if(list.size())
			{
				bool ok = false;
				id = list.front().toInt(&ok);
				if(ok && id>0)
				{
					if(objects_.contains(id))
					{
						UWARN("Object %d already added, a new ID will be generated (new id=%d).", id, Settings::getGeneral_nextObjID());
						id = 0;
					}
				}
				else
				{
					id = 0;
				}
			}
			else
			{
				UERROR("File name doesn't contain \".\" (\"%s\")", filePath.toStdString().c_str());
			}

			const ObjSignature * s = this->addObject(img, id, filePath);
			if(s)
			{
				UINFO("Added object %d (%s)", s->id(), filePath.toStdString().c_str());
				return s;
			}
		}
		else
		{
			UERROR("Could not read image \"%s\"", filePath.toStdString().c_str());
		}
	}
	else
	{
		UERROR("File path is null!?");
	}
	return 0;
}

const ObjSignature * FindObject::addObject(const cv::Mat & image, int id, const QString & filePath)
{
	UASSERT(id >= 0);
	ObjSignature * s = new ObjSignature(id, image, filePath);
	if(!this->addObject(s))
	{
		delete s;
		return 0;
	}
	return s;
}

bool FindObject::addObject(ObjSignature * obj)
{
	UASSERT(obj != 0 && obj->id() >= 0);
	if(obj->id() && objects_.contains(obj->id()))
	{
		UERROR("object with id %d already added!", obj->id());
		return false;
	}
	else if(obj->id() == 0)
	{
		obj->setId(Settings::getGeneral_nextObjID());
	}

	Settings::setGeneral_nextObjID(obj->id()+1);

	objects_.insert(obj->id(), obj);

	return true;
}

void FindObject::removeObject(int id)
{
	if(objects_.contains(id))
	{
		delete objects_.value(id);
		objects_.remove(id);
		clearVocabulary();
	}
}

void FindObject::removeAllObjects()
{
	qDeleteAll(objects_);
	objects_.clear();
	clearVocabulary();
}

void FindObject::addObjectAndUpdate(const cv::Mat & image, int id, const QString & filePath)
{
	const ObjSignature * s = this->addObject(image, id, filePath);
	if(s)
	{
		QList<int> ids;
		ids.push_back(s->id());
		updateObjects(ids);
		updateVocabulary(ids);
	}
}

void FindObject::removeObjectAndUpdate(int id)
{
	if(objects_.contains(id))
	{
		delete objects_.value(id);
		objects_.remove(id);
	}
	updateVocabulary();
}

void FindObject::updateDetectorExtractor()
{
	delete detector_;
	delete extractor_;
	detector_ = Settings::createKeypointDetector();
	extractor_ = Settings::createDescriptorExtractor();
	UASSERT(detector_ != 0 && extractor_ != 0);
}

std::vector<cv::KeyPoint> limitKeypoints(const std::vector<cv::KeyPoint> & keypoints, int maxKeypoints)
{
	std::vector<cv::KeyPoint> kptsKept;
	if(maxKeypoints > 0 && (int)keypoints.size() > maxKeypoints)
	{
		// Sort words by response
		std::multimap<float, int> reponseMap; // <response,id>
		for(unsigned int i = 0; i <keypoints.size(); ++i)
		{
			//Keep track of the data, to be easier to manage the data in the next step
			reponseMap.insert(std::pair<float, int>(fabs(keypoints[i].response), i));
		}

		// Remove them
		std::multimap<float, int>::reverse_iterator iter = reponseMap.rbegin();
		kptsKept.resize(maxKeypoints);
		for(unsigned int k=0; k < kptsKept.size() && iter!=reponseMap.rend(); ++k, ++iter)
		{
			kptsKept[k] = keypoints[iter->second];
		}
	}
	else
	{
		kptsKept = keypoints;
	}
	return kptsKept;
}

void limitKeypoints(std::vector<cv::KeyPoint> & keypoints, cv::Mat & descriptors, int maxKeypoints)
{
	UASSERT((int)keypoints.size() == descriptors.rows);
	std::vector<cv::KeyPoint> kptsKept;
	cv::Mat descriptorsKept;
	if(maxKeypoints > 0 && (int)keypoints.size() > maxKeypoints)
	{
		descriptorsKept = cv::Mat(0, descriptors.cols, descriptors.type());

		// Sort words by response
		std::multimap<float, int> reponseMap; // <response,id>
		for(unsigned int i = 0; i <keypoints.size(); ++i)
		{
			//Keep track of the data, to be easier to manage the data in the next step
			reponseMap.insert(std::pair<float, int>(fabs(keypoints[i].response), i));
		}

		// Remove them
		std::multimap<float, int>::reverse_iterator iter = reponseMap.rbegin();
		kptsKept.resize(maxKeypoints);
		descriptorsKept.reserve(maxKeypoints);
		for(unsigned int k=0; k < kptsKept.size() && iter!=reponseMap.rend(); ++k, ++iter)
		{
			kptsKept[k] = keypoints[iter->second];
			descriptorsKept.push_back(descriptors.row(iter->second));
		}
	}
	keypoints = kptsKept;
	descriptors = descriptorsKept;
	UASSERT_MSG((int)keypoints.size() == descriptors.rows, uFormat("%d vs %d", (int)keypoints.size(), descriptors.rows).c_str());
}

void computeFeatures(
		Feature2D * detector,
		Feature2D * extractor,
		const cv::Mat & image,
		const cv::Mat & mask,
		std::vector<cv::KeyPoint> & keypoints,
		cv::Mat & descriptors,
		int & timeDetection,
		int & timeExtraction)
{
	QTime timeStep;
	timeStep.start();
	keypoints.clear();
	descriptors = cv::Mat();

	int maxFeatures = Settings::getFeature2D_3MaxFeatures();
	if(Settings::currentDetectorType() == Settings::currentDescriptorType())
	{
		detector->detectAndCompute(image, keypoints, descriptors, mask);
		UASSERT_MSG((int)keypoints.size() == descriptors.rows, uFormat("%d vs %d", (int)keypoints.size(), descriptors.rows).c_str());
		if(maxFeatures > 0 && (int)keypoints.size() > maxFeatures)
		{
			limitKeypoints(keypoints, descriptors, maxFeatures);
		}
		timeDetection=timeStep.restart();
		timeExtraction = 0;
	}
	else
	{
		detector->detect(image, keypoints, mask);
		if(maxFeatures > 0 && (int)keypoints.size() > maxFeatures)
		{
			keypoints = limitKeypoints(keypoints, maxFeatures);
		}
		timeDetection=timeStep.restart();

		//Extract descriptors
		try
		{
			extractor->compute(image, keypoints, descriptors);
			UASSERT_MSG((int)keypoints.size() == descriptors.rows, uFormat("%d vs %d", (int)keypoints.size(), descriptors.rows).c_str());
		}
		catch(cv::Exception & e)
		{
			UERROR("Descriptor exception: %s. Maybe some keypoints are invalid "
					"for the selected descriptor extractor.", e.what());
			descriptors = cv::Mat();
			keypoints.clear();
		}
		catch ( const std::exception& e )
		{
			// standard exceptions
			UERROR("Descriptor exception: %s. Maybe some keypoints are invalid "
					"for the selected descriptor extractor.", e.what());
			descriptors = cv::Mat();
			keypoints.clear();
		}
		timeExtraction+=timeStep.restart();
	}

	if( Settings::getFeature2D_SIFT_rootSIFT() &&
		Settings::currentDescriptorType() == "SIFT" &&
		!descriptors.empty())
	{
		UINFO("Performing RootSIFT...");
		// see http://www.pyimagesearch.com/2015/04/13/implementing-rootsift-in-python-and-opencv/
		// apply the Hellinger kernel by first L1-normalizing and taking the
		// square-root
		for(int i=0; i<descriptors.rows; ++i)
		{
			descriptors.row(i) = descriptors.row(i) / cv::sum(descriptors.row(i))[0];
			cv::sqrt(descriptors.row(i), descriptors.row(i));

			// By taking the L1 norm, followed by the square-root, we have
			// already L2 normalized the feature vector and further normalization
			// is not needed.
			//descs /= (np.linalg.norm(descs, axis=1, ord=2) + eps);
		}
	}
}


// taken from ASIFT example https://github.com/Itseez/opencv/blob/master/samples/python2/asift.py
// affine - is an affine transform matrix from skew_img to img
void FindObject::affineSkew(
		float tilt,
		float phi,
		const cv::Mat & image,
		cv::Mat & skewImage,
		cv::Mat & skewMask,
		cv::Mat & Ai)
{
    float h = image.rows;
    float w = image.cols;
    cv::Mat A = cv::Mat::zeros(2,3,CV_32FC1);
    A.at<float>(0,0) = A.at<float>(1,1) = 1;
    skewMask = cv::Mat::ones(h, w, CV_8U) * 255;
    if(phi != 0.0)
    {
        phi = phi*CV_PI/180.0f; // deg2rad
        float s = std::sin(phi);
        float c = std::cos(phi);
        cv::Mat A22 = (cv::Mat_<float>(2, 2) <<
        		c, -s,
        		s, c);
        cv::Mat cornersIn = (cv::Mat_<float>(4, 2) <<
        		0,0,
        		w,0,
        		w,h,
        		0,h);
        cv::Mat cornersOut = cornersIn * A22.t();
        cv::Rect rect = cv::boundingRect(cornersOut.reshape(2,4));
        A = (cv::Mat_<float>(2, 3) <<
				c, -s, -rect.x,
				s, c, -rect.y);
        cv::warpAffine(image, skewImage, A, cv::Size(rect.width, rect.height), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    }
    else
    {
    	skewImage = image;
    }
    if(tilt != 1.0)
    {
        float s = 0.8*std::sqrt(tilt*tilt-1);
        cv::Mat out, out2;
        cv::GaussianBlur(skewImage, out, cv::Size(0, 0), s, 0.01);
        cv::resize(out, out2, cv::Size(0, 0), 1.0/tilt, 1.0, cv::INTER_NEAREST);
        skewImage = out2;
        A.row(0) /= tilt;
    }
    if(phi != 0.0 || tilt != 1.0)
    {
    	cv::Mat mask = skewMask;
        cv::warpAffine(mask, skewMask, A, skewImage.size(), cv::INTER_NEAREST);
    }
    cv::invertAffineTransform(A, Ai);
}

class AffineExtractionThread : public QThread
{
public:
	AffineExtractionThread(
			Feature2D * detector,
			Feature2D * extractor,
			const cv::Mat & image,
			float tilt,
			float phi) :
		detector_(detector),
		extractor_(extractor),
		image_(image),
		tilt_(tilt),
		phi_(phi),
		timeSkewAffine_(0),
		timeDetection_(0),
		timeExtraction_(0),
		timeSubPix_(0)
	{
		UASSERT(detector && extractor);
	}
	const cv::Mat & image() const {return image_;}
	const std::vector<cv::KeyPoint> & keypoints() const {return keypoints_;}
	const cv::Mat & descriptors() const {return descriptors_;}

	int timeSkewAffine() const {return timeSkewAffine_;}
	int timeDetection() const {return timeDetection_;}
	int timeExtraction() const {return timeExtraction_;}
	int timeSubPix() const {return timeSubPix_;}

protected:
	virtual void run()
	{
		QTime timeStep;
		timeStep.start();
		cv::Mat skewImage, skewMask, Ai;
		FindObject::affineSkew(tilt_, phi_, image_, skewImage, skewMask, Ai);
		timeSkewAffine_=timeStep.restart();

		//Detect features
		computeFeatures(
				detector_,
				extractor_,
				skewImage,
				skewMask,
				keypoints_,
				descriptors_,
				timeDetection_,
				timeExtraction_);
		timeStep.start();

		// Transform points to original image coordinates
		for(unsigned int i=0; i<keypoints_.size(); ++i)
		{
			cv::Mat p = (cv::Mat_<float>(3, 1) << keypoints_[i].pt.x, keypoints_[i].pt.y, 1);
			cv::Mat pa = Ai * p;
			keypoints_[i].pt.x = pa.at<float>(0,0);
			keypoints_[i].pt.y = pa.at<float>(1,0);
		}

		if(keypoints_.size() && Settings::getFeature2D_6SubPix())
		{
			// Sub pixel should be done after descriptors extraction
			std::vector<cv::Point2f> corners;
			cv::KeyPoint::convert(keypoints_, corners);
			cv::cornerSubPix(image_,
					corners,
					cv::Size(Settings::getFeature2D_7SubPixWinSize(), Settings::getFeature2D_7SubPixWinSize()),
					cv::Size(-1,-1),
					cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, Settings::getFeature2D_8SubPixIterations(), Settings::getFeature2D_9SubPixEps() ));
			UASSERT(corners.size() == keypoints_.size());
			for(unsigned int i=0; i<corners.size(); ++i)
			{
				keypoints_[i].pt = corners[i];
			}
			timeSubPix_ +=timeStep.restart();
		}
	}
private:
	Feature2D * detector_;
	Feature2D * extractor_;
	cv::Mat image_;
	float tilt_;
	float phi_;
	std::vector<cv::KeyPoint> keypoints_;
	cv::Mat descriptors_;

	int timeSkewAffine_;
	int timeDetection_;
	int timeExtraction_;
	int timeSubPix_;
};

class ExtractFeaturesThread : public QThread
{
public:
	ExtractFeaturesThread(
			Feature2D * detector,
			Feature2D * extractor,
			int objectId,
			const cv::Mat & image) :
		detector_(detector),
		extractor_(extractor),
		objectId_(objectId),
		image_(image),
		timeSkewAffine_(0),
		timeDetection_(0),
		timeExtraction_(0),
		timeSubPix_(0)
	{
		UASSERT(detector && extractor);
		UASSERT_MSG(!image.empty() && image.type() == CV_8UC1,
				uFormat("Image of object %d is null or not type CV_8UC1!?!? (cols=%d, rows=%d, type=%d)",
						objectId, image.cols, image.rows, image.type()).c_str());
	}
	int objectId() const {return objectId_;}
	const cv::Mat & image() const {return image_;}
	const std::vector<cv::KeyPoint> & keypoints() const {return keypoints_;}
	const cv::Mat & descriptors() const {return descriptors_;}

	int timeSkewAffine() const {return timeSkewAffine_;}
	int timeDetection() const {return timeDetection_;}
	int timeExtraction() const {return timeExtraction_;}
	int timeSubPix() const {return timeSubPix_;}

protected:
	virtual void run()
	{
		QTime time;
		time.start();
		UINFO("Extracting descriptors from object %d...", objectId_);

		QTime timeStep;
		timeStep.start();

		if(!Settings::getFeature2D_4Affine())
		{
			computeFeatures(
					detector_,
					extractor_,
					image_,
					cv::Mat(),
					keypoints_,
					descriptors_,
					timeDetection_,
					timeExtraction_);
			timeStep.start();

			if(keypoints_.size())
			{
				UDEBUG("Detected %d features from object %d...", (int)keypoints_.size(), objectId_);
				if(Settings::getFeature2D_6SubPix())
				{
					// Sub pixel should be done after descriptors extraction
					std::vector<cv::Point2f> corners;
					cv::KeyPoint::convert(keypoints_, corners);
					cv::cornerSubPix(image_,
							corners,
							cv::Size(Settings::getFeature2D_7SubPixWinSize(), Settings::getFeature2D_7SubPixWinSize()),
							cv::Size(-1,-1),
							cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, Settings::getFeature2D_8SubPixIterations(), Settings::getFeature2D_9SubPixEps() ));
					UASSERT(corners.size() == keypoints_.size());
					for(unsigned int i=0; i<corners.size(); ++i)
					{
						keypoints_[i].pt = corners[i];
					}
					timeSubPix_ +=timeStep.restart();
				}
			}
			else
			{
				UWARN("no features detected in object %d !?!", objectId_);
			}
		}
		else
		{
			//ASIFT
			std::vector<float> tilts;
			std::vector<float> phis;
			tilts.push_back(1.0f);
			phis.push_back(0.0f);
			int nTilt = Settings::getFeature2D_5AffineCount();
			for(int t=1; t<nTilt; ++t)
			{
				float tilt = std::pow(2.0f, 0.5f*float(t));
				float inc = 72.0f / float(tilt);
				for(float phi=0.0f; phi<180.0f; phi+=inc)
				{
					tilts.push_back(tilt);
					phis.push_back(phi);
				}
			}

			//multi-threaded
			unsigned int threadCounts = Settings::getGeneral_threads();
			if(threadCounts == 0)
			{
				threadCounts = (unsigned int)tilts.size();
			}

			for(unsigned int i=0; i<tilts.size(); i+=threadCounts)
			{
				QVector<AffineExtractionThread*> threads;

				for(unsigned int k=i; k<i+threadCounts && k<tilts.size(); ++k)
				{
					threads.push_back(new AffineExtractionThread(detector_, extractor_, image_, tilts[k], phis[k]));
					threads.back()->start();
				}

				for(int k=0; k<threads.size(); ++k)
				{
					threads[k]->wait();

					keypoints_.insert(keypoints_.end(), threads[k]->keypoints().begin(), threads[k]->keypoints().end());
					descriptors_.push_back(threads[k]->descriptors());

					timeSkewAffine_ += threads[k]->timeSkewAffine();
					timeDetection_ += threads[k]->timeDetection();
					timeExtraction_ += threads[k]->timeExtraction();
					timeSubPix_ += threads[k]->timeSubPix();
				}
			}
		}

		UINFO("%d descriptors extracted from object %d (in %d ms)", descriptors_.rows, objectId_, time.elapsed());
	}
private:
	Feature2D * detector_;
	Feature2D * extractor_;
	int objectId_;
	cv::Mat image_;
	std::vector<cv::KeyPoint> keypoints_;
	cv::Mat descriptors_;

	int timeSkewAffine_;
	int timeDetection_;
	int timeExtraction_;
	int timeSubPix_;
};

void FindObject::updateObjects(const QList<int> & ids)
{
	QList<ObjSignature*> objectsList;
	if(ids.size())
	{
		for(int i=0; i<ids.size(); ++i)
		{
			if(objects_.contains(ids[i]))
			{
				objectsList.push_back(objects_[ids[i]]);
			}
			else
			{
				UERROR("Not found object %d!", ids[i]);
			}
		}
	}
	else
	{
		objectsList = objects_.values();
	}

	if(objectsList.size())
	{
		sessionModified_ = true;
		int threadCounts = Settings::getGeneral_threads();
		if(threadCounts == 0)
		{
			threadCounts = objectsList.size();
		}

		QTime time;
		time.start();

		if(objectsList.size())
		{
			UINFO("Features extraction from %d objects...", objectsList.size());
			for(int i=0; i<objectsList.size(); i+=threadCounts)
			{
				QVector<ExtractFeaturesThread*> threads;
				for(int k=i; k<i+threadCounts && k<objectsList.size(); ++k)
				{
					if(!objectsList.at(k)->image().empty())
					{
						threads.push_back(new ExtractFeaturesThread(detector_, extractor_, objectsList.at(k)->id(), objectsList.at(k)->image()));
						threads.back()->start();
					}
					else
					{
						objects_.value(objectsList.at(k)->id())->setData(std::vector<cv::KeyPoint>(), cv::Mat());
						if(keepImagesInRAM_)
						{
							UERROR("Empty image detected for object %d!? No features can be detected.", objectsList.at(k)->id());

						}
						else
						{
							UWARN("Empty image detected for object %d! No features can be detected. Note that images are in not kept in RAM.", objectsList.at(k)->id());
						}
					}
				}

				for(int j=0; j<threads.size(); ++j)
				{
					threads[j]->wait();

					int id = threads[j]->objectId();

					objects_.value(id)->setData(threads[j]->keypoints(), threads[j]->descriptors());

					if(!keepImagesInRAM_)
					{
						objects_.value(id)->removeImage();
					}
				}
			}
			UINFO("Features extraction from %d objects... done! (%d ms)", objectsList.size(), time.elapsed());
		}
	}
	else
	{
		UINFO("No objects to update...");
	}
}

void FindObject::clearVocabulary()
{
	objectsDescriptors_.clear();
	dataRange_.clear();
	vocabulary_->clear();
}

void FindObject::updateVocabulary(const QList<int> & ids)
{
	int count = 0;
	int dim = -1;
	int type = -1;
	QList<ObjSignature*> objectsList;
	if(ids.size())
	{
		for(int i=0; i<ids.size(); ++i)
		{
			if(objects_.contains(ids[i]))
			{
				objectsList.push_back(objects_[ids[i]]);
			}
			else
			{
				UERROR("Not found object %d!", ids[i]);
			}
		}
		if(vocabulary_->size())
		{
			dim = vocabulary_->dim();
			type = vocabulary_->type();
		}
	}
	else
	{
		clearVocabulary();
		objectsList = objects_.values();
	}

	// Get the total size and verify descriptors
	for(int i=0; i<objectsList.size(); ++i)
	{
		if(!objectsList.at(i)->descriptors().empty())
		{
			if(dim >= 0 && objectsList.at(i)->descriptors().cols != dim)
			{
				UERROR("Descriptors of the objects are not all the same size! Objects "
						"opened must have all the same size (and from the same descriptor extractor).");
				return;
			}
			dim = objectsList.at(i)->descriptors().cols;
			if(type >= 0 && objectsList.at(i)->descriptors().type() != type)
			{
				UERROR("Descriptors of the objects are not all the same type! Objects opened "
						"must have been processed by the same descriptor extractor.");
				return;
			}
			type = objectsList.at(i)->descriptors().type();
			count += objectsList.at(i)->descriptors().rows;
		}
	}

	// Copy data
	if(count)
	{
		UINFO("Updating global descriptors matrix: Objects=%d, total descriptors=%d, dim=%d, type=%d",
				(int)objects_.size(), count, dim, type);
		if(!Settings::getGeneral_invertedSearch())
		{
			if(Settings::getGeneral_threads() == 1)
			{
				// If only one thread, put all descriptors in the same cv::Mat
				int row = 0;
				bool vocabularyEmpty = objectsDescriptors_.size() == 0;
				if(vocabularyEmpty)
				{
					UASSERT(objectsDescriptors_.size() == 0);
					objectsDescriptors_.insert(0, cv::Mat(count, dim, type));
				}
				else
				{
					row = objectsDescriptors_.begin().value().rows;
				}
				for(int i=0; i<objectsList.size(); ++i)
				{
					objectsList[i]->setWords(QMultiMap<int,int>());
					if(objectsList.at(i)->descriptors().rows)
					{
						if(vocabularyEmpty)
						{
							cv::Mat dest(objectsDescriptors_.begin().value(), cv::Range(row, row+objectsList.at(i)->descriptors().rows));
							objectsList.at(i)->descriptors().copyTo(dest);
						}
						else
						{
							UASSERT_MSG(objectsDescriptors_.begin().value().cols == objectsList.at(i)->descriptors().cols,
									uFormat("%d vs %d", objectsDescriptors_.begin().value().cols, objectsList.at(i)->descriptors().cols).c_str());
							UASSERT(objectsDescriptors_.begin().value().type() == objectsList.at(i)->descriptors().type());
							objectsDescriptors_.begin().value().push_back(objectsList.at(i)->descriptors());
						}

						row += objectsList.at(i)->descriptors().rows;
						// dataRange contains the upper_bound for each
						// object (the last descriptors position in the
						// global object descriptors matrix)
						if(objectsList.at(i)->descriptors().rows)
						{
							dataRange_.insert(row-1, objectsList.at(i)->id());
						}
					}
				}
			}
			else
			{
				for(int i=0; i<objectsList.size(); ++i)
				{
					objectsList[i]->setWords(QMultiMap<int,int>());
					objectsDescriptors_.insert(objectsList.at(i)->id(), objectsList.at(i)->descriptors());
				}
			}
		}
		else
		{
			// Inverted index on (vocabulary)
			sessionModified_ = true;
			QTime time;
			time.start();
			bool incremental = Settings::getGeneral_vocabularyIncremental() && !Settings::getGeneral_vocabularyFixed();
			if(incremental)
			{
				UINFO("Creating incremental vocabulary...");
			}
			else if(Settings::getGeneral_vocabularyFixed())
			{
				UINFO("Updating vocabulary correspondences only (vocabulary is fixed)...");
			}
			else
			{
				UINFO("Creating vocabulary...");
			}
			QTime localTime;
			localTime.start();
			int updateVocabularyMinWords = Settings::getGeneral_vocabularyUpdateMinWords();
			int addedWords = 0;
			for(int i=0; i<objectsList.size(); ++i)
			{
				UASSERT(objectsList[i]->descriptors().rows == (int)objectsList[i]->keypoints().size());
				QMultiMap<int, int> words = vocabulary_->addWords(objectsList[i]->descriptors(), objectsList.at(i)->id());
				objectsList[i]->setWords(words);
				addedWords += words.uniqueKeys().size();
				bool updated = false;
				if(incremental && addedWords && addedWords >= updateVocabularyMinWords)
				{
					vocabulary_->update();
					addedWords = 0;
					updated = true;
				}
				UINFO("Object %d, %d words from %d descriptors (%d words, %d ms) %s",
						objectsList[i]->id(),
						words.uniqueKeys().size(),
						objectsList[i]->descriptors().rows,
						vocabulary_->size(),
						localTime.restart(),
						updated?"updated":"");
			}
			if(addedWords && !Settings::getGeneral_vocabularyFixed())
			{
				vocabulary_->update();
			}

			if(incremental)
			{
				UINFO("Creating incremental vocabulary... done! size=%d (%d ms)", vocabulary_->size(), time.elapsed());
			}
			else if(Settings::getGeneral_vocabularyFixed())
			{
				UINFO("Updating vocabulary correspondences only (vocabulary is fixed)... done! size=%d (%d ms)", vocabulary_->size(), time.elapsed());
			}
			else
			{
				UINFO("Creating vocabulary... done! size=%d (%d ms)", vocabulary_->size(), time.elapsed());
			}
		}
	}
}

class SearchThread: public QThread
{
public:
	SearchThread(Vocabulary * vocabulary, int objectId, const cv::Mat * descriptors, const QMultiMap<int, int> * sceneWords) :
		vocabulary_(vocabulary),
		objectId_(objectId),
		descriptors_(descriptors),
		sceneWords_(sceneWords),
		minMatchedDistance_(-1.0f),
		maxMatchedDistance_(-1.0f)
	{
		UASSERT(descriptors);
	}
	virtual ~SearchThread() {}

	int getObjectId() const {return objectId_;}
	float getMinMatchedDistance() const {return minMatchedDistance_;}
	float getMaxMatchedDistance() const {return maxMatchedDistance_;}
	const QMultiMap<int, int> & getMatches() const {return matches_;}

protected:
	virtual void run()
	{
		//QTime time;
		//time.start();

		cv::Mat results;
		cv::Mat dists;

		//match objects to scene
		int k = Settings::getNearestNeighbor_3nndrRatioUsed()?2:1;
		results = cv::Mat(descriptors_->rows, k, CV_32SC1); // results index
		dists = cv::Mat(descriptors_->rows, k, CV_32FC1); // Distance results are CV_32FC1
		vocabulary_->search(*descriptors_, results, dists, k);

		// PROCESS RESULTS
		// Get all matches for each object
		for(int i=0; i<dists.rows; ++i)
		{
			// Check if this descriptor matches with those of the objects
			bool matched = false;

			if(Settings::getNearestNeighbor_3nndrRatioUsed() &&
			   dists.at<float>(i,0) <= Settings::getNearestNeighbor_4nndrRatio() * dists.at<float>(i,1))
			{
				matched = true;
			}
			if((matched || !Settings::getNearestNeighbor_3nndrRatioUsed()) &&
			   Settings::getNearestNeighbor_5minDistanceUsed())
			{
				if(dists.at<float>(i,0) <= Settings::getNearestNeighbor_6minDistance())
				{
					matched = true;
				}
				else
				{
					matched = false;
				}
			}
			if(!matched && !Settings::getNearestNeighbor_3nndrRatioUsed() && !Settings::getNearestNeighbor_5minDistanceUsed())
			{
				matched = true; // no criterion, match to the nearest descriptor
			}
			if(minMatchedDistance_ == -1 || minMatchedDistance_ > dists.at<float>(i,0))
			{
				minMatchedDistance_ = dists.at<float>(i,0);
			}
			if(maxMatchedDistance_ == -1 || maxMatchedDistance_ < dists.at<float>(i,0))
			{
				maxMatchedDistance_ = dists.at<float>(i,0);
			}

			int wordId = results.at<int>(i,0);
			if(matched && sceneWords_->count(wordId) == 1)
			{
				matches_.insert(i, sceneWords_->value(wordId));
				matches_.insert(i, results.at<int>(i,0));
			}
		}

		//UINFO("Search Object %d time=%d ms", objectIndex_, time.elapsed());
	}
private:
	Vocabulary * vocabulary_; // would be const but flann search() method is not const!?
	int objectId_;
	const cv::Mat * descriptors_;
	const QMultiMap<int, int> * sceneWords_; // <word id, keypoint indexes>

	float minMatchedDistance_;
	float maxMatchedDistance_;
	QMultiMap<int, int> matches_;
};

class HomographyThread: public QThread
{
public:
	HomographyThread(
			const QMultiMap<int, int> * matches, // <object, scene>
			int objectId,
			const std::vector<cv::KeyPoint> * kptsA,
			const std::vector<cv::KeyPoint> * kptsB,
			const cv::Mat & imageA,   // image only required if opticalFlow is on
			const cv::Mat & imageB) : // image only required if opticalFlow is on
				matches_(matches),
				objectId_(objectId),
				kptsA_(kptsA),
				kptsB_(kptsB),
				imageA_(imageA),
				imageB_(imageB),
				code_(DetectionInfo::kRejectedUndef)
	{
		UASSERT(matches && kptsA && kptsB);
	}
	virtual ~HomographyThread() {}

	int getObjectId() const {return objectId_;}
	const std::vector<int> & getIndexesA() const {return indexesA_;}
	const std::vector<int> & getIndexesB() const {return indexesB_;}
	const std::vector<uchar> & getOutlierMask() const {return outlierMask_;}
	QMultiMap<int, int> getInliers() const {return inliers_;}
	QMultiMap<int, int> getOutliers() const {return outliers_;}
	const cv::Mat & getHomography() const {return h_;}
	DetectionInfo::RejectedCode rejectedCode() const {return code_;}

protected:
	virtual void run()
	{
		//QTime time;
		//time.start();

		std::vector<cv::Point2f> mpts_1(matches_->size());
		std::vector<cv::Point2f> mpts_2(matches_->size());
		indexesA_.resize(matches_->size());
		indexesB_.resize(matches_->size());

		UDEBUG("Fill matches...");
		int j=0;
		for(QMultiMap<int, int>::const_iterator iter = matches_->begin(); iter!=matches_->end(); ++iter)
		{
			UASSERT_MSG(iter.key() < (int)kptsA_->size(), uFormat("key=%d size=%d", iter.key(),(int)kptsA_->size()).c_str());
			UASSERT_MSG(iter.value() < (int)kptsB_->size(), uFormat("key=%d size=%d", iter.value(),(int)kptsB_->size()).c_str());
			mpts_1[j] = kptsA_->at(iter.key()).pt;
			indexesA_[j] = iter.key();
			mpts_2[j] = kptsB_->at(iter.value()).pt;
			indexesB_[j] = iter.value();
			++j;
		}

		if((int)mpts_1.size() >= Settings::getHomography_minimumInliers())
		{
			if(Settings::getHomography_opticalFlow())
			{
				UASSERT(!imageA_.empty() && !imageB_.empty());

				cv::Mat imageA = imageA_;
				cv::Mat imageB = imageB_;
				if(imageA_.cols < imageB_.cols && imageA_.rows < imageB_.rows)
				{
					// padding, optical flow wants images of the same size
					imageA = cv::Mat::zeros(imageB_.size(), imageA_.type());
					imageA_.copyTo(imageA(cv::Rect(0,0,imageA_.cols, imageA_.rows)));
				}
				if(imageA.size() == imageB.size())
				{
					UDEBUG("Optical flow...");
					//refine matches
					std::vector<unsigned char> status;
					std::vector<float> err;
					cv::calcOpticalFlowPyrLK(
							imageA,
							imageB_,
							mpts_1,
							mpts_2,
							status,
							err,
							cv::Size(Settings::getHomography_opticalFlowWinSize(), Settings::getHomography_opticalFlowWinSize()),
							Settings::getHomography_opticalFlowMaxLevel(),
							cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, Settings::getHomography_opticalFlowIterations(), Settings::getHomography_opticalFlowEps()),
							cv::OPTFLOW_LK_GET_MIN_EIGENVALS | cv::OPTFLOW_USE_INITIAL_FLOW, 1e-4);
				}
				else
				{
					UERROR("Object's image should be less/equal size of the scene image to use Optical Flow.");
				}
			}

			UDEBUG("Find homography... begin");
#if CV_MAJOR_VERSION < 3
			h_ = findHomography(mpts_1,
					mpts_2,
					Settings::getHomographyMethod(),
					Settings::getHomography_ransacReprojThr(),
					outlierMask_);
#else
			h_ = findHomography(mpts_1,
					mpts_2,
					Settings::getHomographyMethod(),
					Settings::getHomography_ransacReprojThr(),
					outlierMask_,
					Settings::getHomography_maxIterations(),
					Settings::getHomography_confidence());
#endif
			UDEBUG("Find homography... end");

			UASSERT(outlierMask_.size() == 0 || outlierMask_.size() == mpts_1.size());
			for(unsigned int k=0; k<mpts_1.size();++k)
			{
				if(outlierMask_.size() && outlierMask_.at(k))
				{
					inliers_.insert(indexesA_[k], indexesB_[k]);
				}
				else
				{
					outliers_.insert(indexesA_[k], indexesB_[k]);
				}
			}

			if(inliers_.size() == (int)outlierMask_.size() && !h_.empty())
			{
				if(Settings::getHomography_ignoreWhenAllInliers() || cv::countNonZero(h_) < 1)
				{
					// ignore homography when all features are inliers
					h_ = cv::Mat();
					code_ = DetectionInfo::kRejectedAllInliers;
				}
			}
		}
		else
		{
			code_ = DetectionInfo::kRejectedLowMatches;
		}

		//UINFO("Homography Object %d time=%d ms", objectIndex_, time.elapsed());
	}
private:
	const QMultiMap<int, int> * matches_;
	int objectId_;
	const std::vector<cv::KeyPoint> * kptsA_;
	const std::vector<cv::KeyPoint> * kptsB_;
	cv::Mat imageA_;
	cv::Mat imageB_;
	DetectionInfo::RejectedCode code_;

	std::vector<int> indexesA_;
	std::vector<int> indexesB_;
	std::vector<uchar> outlierMask_;
	QMultiMap<int, int> inliers_;
	QMultiMap<int, int> outliers_;
	cv::Mat h_;
};

void FindObject::detect(const cv::Mat & image)
{
	QTime time;
	time.start();
	DetectionInfo info;
	this->detect(image, info);

	if(info.objDetected_.size() > 1)
	{
		UINFO("(%s) %d objects detected! (%d ms)",
				QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str(),
				(int)info.objDetected_.size(),
				time.elapsed());
	}
	else if(info.objDetected_.size() == 1)
	{
		UINFO("(%s) Object %d detected! (%d ms)",
				QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str(),
				(int)info.objDetected_.begin().key(),
				time.elapsed());
	}
	else if(Settings::getGeneral_sendNoObjDetectedEvents())
	{
		UINFO("(%s) No objects detected. (%d ms)",
				QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str(),
				time.elapsed());
	}

	if(info.objDetected_.size() > 0 || Settings::getGeneral_sendNoObjDetectedEvents())
	{
		Q_EMIT objectsFound(info);
	}
}

bool FindObject::detect(const cv::Mat & image, find_object::DetectionInfo & info) const
{
	QTime totalTime;
	totalTime.start();

	// reset statistics
	info = DetectionInfo();

	bool success = false;
	if(!image.empty())
	{
		//Convert to grayscale
		cv::Mat grayscaleImg;
		if(image.channels() != 1 || image.depth() != CV_8U)
		{
			cv::cvtColor(image, grayscaleImg, cv::COLOR_BGR2GRAY);
		}
		else
		{
			grayscaleImg =  image;
		}

		// DETECT FEATURES AND EXTRACT DESCRIPTORS
		UDEBUG("DETECT FEATURES AND EXTRACT DESCRIPTORS FROM THE SCENE");
		ExtractFeaturesThread extractThread(detector_, extractor_, -1, grayscaleImg);
		extractThread.start();
		extractThread.wait();
		info.sceneKeypoints_ = extractThread.keypoints();
		info.sceneDescriptors_ = extractThread.descriptors();
		UASSERT_MSG((int)extractThread.keypoints().size() == extractThread.descriptors().rows, uFormat("%d vs %d", (int)extractThread.keypoints().size(), extractThread.descriptors().rows).c_str());
		info.timeStamps_.insert(DetectionInfo::kTimeKeypointDetection, extractThread.timeDetection());
		info.timeStamps_.insert(DetectionInfo::kTimeDescriptorExtraction, extractThread.timeExtraction());
		info.timeStamps_.insert(DetectionInfo::kTimeSubPixelRefining, extractThread.timeSubPix());
		info.timeStamps_.insert(DetectionInfo::kTimeSkewAffine, extractThread.timeSkewAffine());

		bool consistentNNData = (vocabulary_->size()!=0 && vocabulary_->wordToObjects().begin().value()!=-1 && Settings::getGeneral_invertedSearch()) ||
								((vocabulary_->size()==0 || vocabulary_->wordToObjects().begin().value()==-1) && !Settings::getGeneral_invertedSearch());

		bool descriptorsValid = !Settings::getGeneral_invertedSearch() &&
								!objectsDescriptors_.empty() &&
								objectsDescriptors_.begin().value().cols == info.sceneDescriptors_.cols &&
								objectsDescriptors_.begin().value().type() == info.sceneDescriptors_.type();

		bool vocabularyValid = Settings::getGeneral_invertedSearch() &&
								vocabulary_->size() &&
								!vocabulary_->indexedDescriptors().empty() &&
								vocabulary_->indexedDescriptors().cols == info.sceneDescriptors_.cols &&
								(vocabulary_->indexedDescriptors().type() == info.sceneDescriptors_.type() ||
										(Settings::getNearestNeighbor_7ConvertBinToFloat() && vocabulary_->indexedDescriptors().type() == CV_32FC1));

		// COMPARE
		UDEBUG("COMPARE");
		if((descriptorsValid || vocabularyValid) &&
			info.sceneKeypoints_.size() &&
		    consistentNNData)
		{
			success = true;
			QTime time;
			time.start();

			QMultiMap<int, int> words;

			if(!Settings::getGeneral_invertedSearch())
			{
				vocabulary_->clear();
				// CREATE INDEX for the scene
				UDEBUG("CREATE INDEX FOR THE SCENE");
				words = vocabulary_->addWords(info.sceneDescriptors_, -1);
				vocabulary_->update();
				info.timeStamps_.insert(DetectionInfo::kTimeIndexing, time.restart());
				info.sceneWords_ = words;
			}

			for(QMap<int, ObjSignature*>::const_iterator iter=objects_.begin(); iter!=objects_.end(); ++iter)
			{
				info.matches_.insert(iter.key(), QMultiMap<int, int>());
			}

			if(Settings::getGeneral_invertedSearch() || Settings::getGeneral_threads() == 1)
			{
				cv::Mat results;
				cv::Mat dists;
				// DO NEAREST NEIGHBOR
				UDEBUG("DO NEAREST NEIGHBOR");
				int k = Settings::getNearestNeighbor_3nndrRatioUsed()?2:1;
				if(!Settings::getGeneral_invertedSearch())
				{
					//match objects to scene
					results = cv::Mat(objectsDescriptors_.begin().value().rows, k, CV_32SC1); // results index
					dists = cv::Mat(objectsDescriptors_.begin().value().rows, k, CV_32FC1); // Distance results are CV_32FC1
					vocabulary_->search(objectsDescriptors_.begin().value(), results, dists, k);
				}
				else
				{
					//match scene to objects
					results = cv::Mat(info.sceneDescriptors_.rows, k, CV_32SC1); // results index
					dists = cv::Mat(info.sceneDescriptors_.rows, k, CV_32FC1); // Distance results are CV_32FC1
					vocabulary_->search(info.sceneDescriptors_, results, dists, k);
				}

				// PROCESS RESULTS
				UDEBUG("PROCESS RESULTS");
				// Get all matches for each object
				for(int i=0; i<dists.rows; ++i)
				{
					// Check if this descriptor matches with those of the objects
					bool matched = false;

					if(Settings::getNearestNeighbor_3nndrRatioUsed() &&
					   dists.at<float>(i,0) <= Settings::getNearestNeighbor_4nndrRatio() * dists.at<float>(i,1))
					{
						matched = true;
					}
					if((matched || !Settings::getNearestNeighbor_3nndrRatioUsed()) &&
					   Settings::getNearestNeighbor_5minDistanceUsed())
					{
						if(dists.at<float>(i,0) <= Settings::getNearestNeighbor_6minDistance())
						{
							matched = true;
						}
						else
						{
							matched = false;
						}
					}
					if(!matched &&
					   !Settings::getNearestNeighbor_3nndrRatioUsed() &&
					   !Settings::getNearestNeighbor_5minDistanceUsed() &&
					   dists.at<float>(i,0) >= 0.0f)
					{
						matched = true; // no criterion, match to the nearest descriptor
					}
					if(info.minMatchedDistance_ == -1 || info.minMatchedDistance_ > dists.at<float>(i,0))
					{
						info.minMatchedDistance_ = dists.at<float>(i,0);
					}
					if(info.maxMatchedDistance_ == -1 || info.maxMatchedDistance_ < dists.at<float>(i,0))
					{
						info.maxMatchedDistance_ = dists.at<float>(i,0);
					}

					if(matched)
					{
						int wordId = results.at<int>(i,0);
						if(Settings::getGeneral_invertedSearch())
						{
							info.sceneWords_.insertMulti(wordId, i);
							QList<int> objIds = vocabulary_->wordToObjects().values(wordId);
							for(int j=0; j<objIds.size(); ++j)
							{
								// just add unique matches
								if(vocabulary_->wordToObjects().count(wordId, objIds[j]) == 1)
								{
									info.matches_.find(objIds[j]).value().insert(objects_.value(objIds[j])->words().value(wordId), i);
								}
							}
						}
						else
						{
							QMap<int, int>::const_iterator iter = dataRange_.lowerBound(i);
							int objectId = iter.value();
							int fisrtObjectDescriptorIndex = (iter == dataRange_.begin())?0:(--iter).key()+1;
							int objectDescriptorIndex = i - fisrtObjectDescriptorIndex;

							if(words.count(wordId) == 1)
							{
								info.matches_.find(objectId).value().insert(objectDescriptorIndex, words.value(wordId));
							}
						}
					}
				}
			}
			else
			{
				//multi-threaded, match objects to scene
				UDEBUG("MULTI-THREADED, MATCH OBJECTS TO SCENE");
				int threadCounts = Settings::getGeneral_threads();
				if(threadCounts == 0)
				{
					threadCounts = (int)objectsDescriptors_.size();
				}

				QList<int> objectsDescriptorsId = objectsDescriptors_.keys();
				QList<cv::Mat> objectsDescriptorsMat = objectsDescriptors_.values();
				for(int j=0; j<objectsDescriptorsMat.size(); j+=threadCounts)
				{
					QVector<SearchThread*> threads;

					for(int k=j; k<j+threadCounts && k<objectsDescriptorsMat.size(); ++k)
					{
						threads.push_back(new SearchThread(vocabulary_, objectsDescriptorsId[k], &objectsDescriptorsMat[k], &words));
						threads.back()->start();
					}

					for(int k=0; k<threads.size(); ++k)
					{
						threads[k]->wait();
						info.matches_[threads[k]->getObjectId()] = threads[k]->getMatches();

						if(info.minMatchedDistance_ == -1 || info.minMatchedDistance_ > threads[k]->getMinMatchedDistance())
						{
							info.minMatchedDistance_ = threads[k]->getMinMatchedDistance();
						}
						if(info.maxMatchedDistance_ == -1 || info.maxMatchedDistance_ < threads[k]->getMaxMatchedDistance())
						{
							info.maxMatchedDistance_ = threads[k]->getMaxMatchedDistance();
						}
						delete threads[k];
					}

				}
			}

			info.timeStamps_.insert(DetectionInfo::kTimeMatching, time.restart());

			// Homographies
			if(Settings::getHomography_homographyComputed())
			{
				// HOMOGRAPHY
				UDEBUG("COMPUTE HOMOGRAPHY");
				int threadCounts = Settings::getGeneral_threads();
				if(threadCounts == 0)
				{
					threadCounts = info.matches_.size();
				}
				QList<int> matchesId = info.matches_.keys();
				QList<QMultiMap<int, int> > matchesList = info.matches_.values();
				for(int i=0; i<matchesList.size(); i+=threadCounts)
				{
					UDEBUG("Processing matches %d/%d", i+1, matchesList.size());

					QVector<HomographyThread*> threads;

					UDEBUG("Creating/Starting homography threads (%d)...", threadCounts);
					for(int k=i; k<i+threadCounts && k<matchesList.size(); ++k)
					{
						int objectId = matchesId[k];
						UASSERT(objects_.contains(objectId));
						threads.push_back(new HomographyThread(
								&matchesList[k],
								objectId,
								&objects_.value(objectId)->keypoints(),
								&info.sceneKeypoints_,
								objects_.value(objectId)->image(),
								grayscaleImg));
						threads.back()->start();
					}
					UDEBUG("Started homography threads");

					for(int j=0; j<threads.size(); ++j)
					{
						threads[j]->wait();
						UDEBUG("Processing results of homography thread %d", j);

						int id = threads[j]->getObjectId();
						QTransform hTransform;
						DetectionInfo::RejectedCode code = DetectionInfo::kRejectedUndef;
						if(threads[j]->getHomography().empty())
						{
							code = threads[j]->rejectedCode();
						}
						if(code == DetectionInfo::kRejectedUndef &&
						   threads[j]->getInliers().size() < Settings::getHomography_minimumInliers()	)
						{
							code = DetectionInfo::kRejectedLowInliers;
						}
						if(code == DetectionInfo::kRejectedUndef)
						{
							const cv::Mat & H = threads[j]->getHomography();
							UASSERT(H.cols == 3 && H.rows == 3 && H.type()==CV_64FC1);
							hTransform = QTransform(
								H.at<double>(0,0), H.at<double>(1,0), H.at<double>(2,0),
								H.at<double>(0,1), H.at<double>(1,1), H.at<double>(2,1),
								H.at<double>(0,2), H.at<double>(1,2), H.at<double>(2,2));

							// is homography valid?
							// Here we use mapToScene() from QGraphicsItem instead
							// of QTransform::map() because if the homography is not valid,
							// huge errors are set by the QGraphicsItem and not by QTransform::map();
							UASSERT(objects_.contains(id));
							QRectF objectRect = objects_.value(id)->rect();
							QGraphicsRectItem item(objectRect);
							item.setTransform(hTransform);
							QPolygonF rectH = item.mapToScene(item.rect());

							// If a point is outside of 2x times the surface of the scene, homography is invalid.
							for(int p=0; p<rectH.size(); ++p)
							{
								if((rectH.at(p).x() < -image.cols && rectH.at(p).x() < -objectRect.width()) ||
								   (rectH.at(p).x() > image.cols*2  && rectH.at(p).x() > objectRect.width()*2) ||
								   (rectH.at(p).y() < -image.rows  && rectH.at(p).x() < -objectRect.height()) ||
								   (rectH.at(p).y() > image.rows*2  && rectH.at(p).x() > objectRect.height()*2))
								{
									code= DetectionInfo::kRejectedNotValid;
									break;
								}
							}

							// angle
							if(code == DetectionInfo::kRejectedUndef &&
							   Settings::getHomography_minAngle() > 0)
							{
								for(int a=0; a<rectH.size(); ++a)
								{
									//  Find the smaller angle
									QLineF ab(rectH.at(a).x(), rectH.at(a).y(), rectH.at((a+1)%4).x(), rectH.at((a+1)%4).y());
									QLineF cb(rectH.at((a+1)%4).x(), rectH.at((a+1)%4).y(), rectH.at((a+2)%4).x(), rectH.at((a+2)%4).y());
									float angle =  ab.angle(cb);
									float minAngle = (float)Settings::getHomography_minAngle();
									if(angle < minAngle ||
									   angle > 180.0-minAngle)
									{
										code = DetectionInfo::kRejectedByAngle;
										break;
									}
								}
							}

							// multi detection
							if(code == DetectionInfo::kRejectedUndef &&
							   Settings::getGeneral_multiDetection())
							{
								int distance = Settings::getGeneral_multiDetectionRadius(); // in pixels
								// Get the outliers and recompute homography with them
								matchesList.push_back(threads[j]->getOutliers());
								matchesId.push_back(id);

								// compute distance from previous added same objects...
								QMultiMap<int, QTransform>::iterator objIter = info.objDetected_.find(id);
								for(;objIter!=info.objDetected_.end() && objIter.key() == id; ++objIter)
								{
									qreal dx = objIter.value().m31() - hTransform.m31();
									qreal dy = objIter.value().m32() - hTransform.m32();
									int d = (int)sqrt(dx*dx + dy*dy);
									if(d < distance)
									{
										distance = d;
									}
								}

								if(distance < Settings::getGeneral_multiDetectionRadius())
								{
									code = DetectionInfo::kRejectedSuperposed;
								}
							}

							// Corners visible
							if(code == DetectionInfo::kRejectedUndef &&
							   Settings::getHomography_allCornersVisible())
							{
								// Now verify if all corners are in the scene
								QRectF sceneRect(0,0,image.cols, image.rows);
								for(int p=0; p<rectH.size(); ++p)
								{
									if(!sceneRect.contains(QPointF(rectH.at(p).x(), rectH.at(p).y())))
									{
										code = DetectionInfo::kRejectedCornersOutside;
										break;
									}
								}
							}
						}

						if(code == DetectionInfo::kRejectedUndef)
						{
							// Accepted!
							info.objDetected_.insert(id, hTransform);
							info.objDetectedSizes_.insert(id, objects_.value(id)->rect().size());
							info.objDetectedInliers_.insert(id, threads[j]->getInliers());
							info.objDetectedOutliers_.insert(id, threads[j]->getOutliers());
							info.objDetectedInliersCount_.insert(id, threads[j]->getInliers().size());
							info.objDetectedOutliersCount_.insert(id, threads[j]->getOutliers().size());
							info.objDetectedFilePaths_.insert(id, objects_.value(id)->filePath());
						}
						else
						{
							//Rejected!
							info.rejectedInliers_.insert(id, threads[j]->getInliers());
							info.rejectedOutliers_.insert(id, threads[j]->getOutliers());
							info.rejectedCodes_.insert(id, code);
						}
					}
					UDEBUG("Processed matches %d", i+1);
				}
				info.timeStamps_.insert(DetectionInfo::kTimeHomography, time.restart());
			}
		}
		else if((descriptorsValid || vocabularyValid) && info.sceneKeypoints_.size())
		{
			UWARN("Cannot search, objects must be updated");
		}
		else if(info.sceneKeypoints_.size() == 0)
		{
			// Accept but warn the user
			UWARN("No features detected in the scene!?!");
			success = true;
		}
	}

	info.timeStamps_.insert(DetectionInfo::kTimeTotal, totalTime.elapsed());

	return success;
}

} // namespace find_object
