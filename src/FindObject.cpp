/*
 * FindObject.cpp
 *
 *  Created on: 2014-07-30
 *      Author: mathieu
 */

#include "find_object/FindObject.h"
#include "find_object/Settings.h"
#include "find_object/utilite/ULogger.h"

#include "ObjSignature.h"
#include "utilite/UDirectory.h"
#include "Vocabulary.h"

#include <QtCore/QThread>
#include <QtCore/QFileInfo>
#include <QtCore/QStringList>
#include <QtCore/QTime>
#include <stdio.h>

FindObject::FindObject(QObject * parent) :
	QObject(parent),
	vocabulary_(new Vocabulary()),
	detector_(Settings::createKeypointDetector()),
	extractor_(Settings::createDescriptorExtractor()),
	minMatchedDistance_(-1),
	maxMatchedDistance_(-1)
{
	Q_ASSERT(detector_ != 0 && extractor_ != 0);
}

FindObject::~FindObject() {
	delete detector_;
	delete extractor_;
	delete vocabulary_;
	objectsDescriptors_.clear();
}

int FindObject::loadObjects(const QString & dirPath)
{
	int loadedObjects = 0;
	QString formats = Settings::getGeneral_imageFormats().remove('*').remove('.');
	UDirectory dir(dirPath.toStdString(), formats.toStdString());
	if(dir.isValid())
	{
		const std::list<std::string> & names = dir.getFileNames(); // sorted in natural order
		for(std::list<std::string>::const_iterator iter=names.begin(); iter!=names.end(); ++iter)
		{
			this->addObject((dirPath.toStdString()+dir.separator()+*iter).c_str());
		}
		if(names.size())
		{
			this->updateObjects();
			this->updateVocabulary();
		}
		loadedObjects = (int)names.size();
	}
	return loadedObjects;
}

const ObjSignature * FindObject::addObject(const QString & filePath)
{
	UINFO("Load file %s", filePath.toStdString().c_str());
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
			return this->addObject(img, id);
		}
	}
	return 0;
}

const ObjSignature * FindObject::addObject(const cv::Mat & image, int id)
{
	Q_ASSERT(id >= 0);
	ObjSignature * s = new ObjSignature(id, image);
	if(!this->addObject(s))
	{
		delete s;
		return 0;
	}
	return s;
}

bool FindObject::addObject(ObjSignature * obj)
{
	Q_ASSERT(obj != 0 && obj->id() >= 0);
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
	clearVocabulary();

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

void FindObject::updateDetectorExtractor()
{
	delete detector_;
	delete extractor_;
	detector_ = Settings::createKeypointDetector();
	extractor_ = Settings::createDescriptorExtractor();
	Q_ASSERT(detector_ != 0 && extractor_ != 0);
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

class ExtractFeaturesThread : public QThread
{
public:
	ExtractFeaturesThread(int objectId, const cv::Mat & image) :
		objectId_(objectId),
		image_(image)
	{

	}
	int objectId() const {return objectId_;}
	const cv::Mat & image() const {return image_;}
	const std::vector<cv::KeyPoint> & keypoints() const {return keypoints_;}
	const cv::Mat & descriptors() const {return descriptors_;}

protected:
	virtual void run()
	{
		QTime time;
		time.start();
		UINFO("Extracting descriptors from object %d...", objectId_);
		KeypointDetector * detector = Settings::createKeypointDetector();
		keypoints_.clear();
		descriptors_ = cv::Mat();
		detector->detect(image_, keypoints_);
		delete detector;

		if(keypoints_.size())
		{
			int maxFeatures = Settings::getFeature2D_3MaxFeatures();
			if(maxFeatures > 0 && (int)keypoints_.size() > maxFeatures)
			{
				int previousCount = (int)keypoints_.size();
				keypoints_ = limitKeypoints(keypoints_, maxFeatures);
				UINFO("obj=%d, %d keypoints removed, (kept %d), min/max response=%f/%f", objectId_, previousCount-(int)keypoints_.size(), (int)keypoints_.size(), keypoints_.size()?keypoints_.back().response:0.0f, keypoints_.size()?keypoints_.front().response:0.0f);
			}

			DescriptorExtractor * extractor = Settings::createDescriptorExtractor();
			extractor->compute(image_, keypoints_, descriptors_);
			delete extractor;
			if((int)keypoints_.size() != descriptors_.rows)
			{
				UERROR("obj=%d kpt=%d != descriptors=%d", objectId_, (int)keypoints_.size(), descriptors_.rows);
			}
		}
		else
		{
			UWARN("no features detected in object %d !?!", objectId_);
		}
		UINFO("%d descriptors extracted from object %d (in %d ms)", descriptors_.rows, objectId_, time.elapsed());
	}
private:
	int objectId_;
	cv::Mat image_;
	std::vector<cv::KeyPoint> keypoints_;
	cv::Mat descriptors_;
};

void FindObject::updateObjects()
{
	if(objects_.size())
	{
		int threadCounts = Settings::getGeneral_threads();
		if(threadCounts == 0)
		{
			threadCounts = objects_.size();
		}

		QTime time;
		time.start();
		UINFO("Features extraction from %d objects...", objects_.size());
		QList<ObjSignature*> objectsList = objects_.values();
		for(int i=0; i<objectsList.size(); i+=threadCounts)
		{
			QVector<ExtractFeaturesThread*> threads;
			for(int k=i; k<i+threadCounts && k<objectsList.size(); ++k)
			{
				threads.push_back(new ExtractFeaturesThread(objectsList.at(k)->id(), objectsList.at(k)->image()));
				threads.back()->start();
			}

			for(int j=0; j<threads.size(); ++j)
			{
				threads[j]->wait();

				int id = threads[j]->objectId();

				objects_.value(id)->setData(
						threads[j]->keypoints(),
						threads[j]->descriptors(),
						Settings::currentDetectorType(),
						Settings::currentDescriptorType());
			}
		}
		UINFO("Features extraction from %d objects... done! (%d ms)", objects_.size(), time.elapsed());
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

void FindObject::updateVocabulary()
{
	clearVocabulary();
	int count = 0;
	int dim = -1;
	int type = -1;
	// Get the total size and verify descriptors
	QList<ObjSignature*> objectsList = objects_.values();
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
		if(Settings::getGeneral_invertedSearch() || Settings::getGeneral_threads() == 1)
		{
			// If only one thread, put all descriptors in the same cv::Mat
			objectsDescriptors_.insert(0, cv::Mat(count, dim, type));
			int row = 0;
			for(int i=0; i<objectsList.size(); ++i)
			{
				if(objectsList.at(i)->descriptors().rows)
				{
					cv::Mat dest(objectsDescriptors_.begin().value(), cv::Range(row, row+objectsList.at(i)->descriptors().rows));
					objectsList.at(i)->descriptors().copyTo(dest);
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

			if(Settings::getGeneral_invertedSearch())
			{
				QTime time;
				time.start();
				bool incremental = Settings::getGeneral_vocabularyIncremental();
				if(incremental)
				{
					UINFO("Creating incremental vocabulary...");
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
					QMultiMap<int, int> words = vocabulary_->addWords(objectsList[i]->descriptors(), objectsList.at(i)->id(), incremental);
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
				if(addedWords)
				{
					vocabulary_->update();
				}

				if(incremental)
				{
					UINFO("Creating incremental vocabulary... done! size=%d (%d ms)", vocabulary_->size(), time.elapsed());
				}
				else
				{
					UINFO("Creating vocabulary... done! size=%d (%d ms)", vocabulary_->size(), time.elapsed());
				}
			}
		}
		else
		{
			for(int i=0; i<objectsList.size(); ++i)
			{
				objectsDescriptors_.insert(objectsList.at(i)->id(), objectsList.at(i)->descriptors());
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
		Q_ASSERT(index && descriptors);
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
			const std::vector<cv::KeyPoint> * kptsB) :
				matches_(matches),
				objectId_(objectId),
				kptsA_(kptsA),
				kptsB_(kptsB)
	{
		Q_ASSERT(matches && kptsA && kptsB);
	}
	virtual ~HomographyThread() {}

	int getObjectId() const {return objectId_;}
	const std::vector<int> & getIndexesA() const {return indexesA_;}
	const std::vector<int> & getIndexesB() const {return indexesB_;}
	const std::vector<uchar> & getOutlierMask() const {return outlierMask_;}
	QMultiMap<int, int> getInliers() const {return inliers_;}
	QMultiMap<int, int> getOutliers() const {return outliers_;}
	const cv::Mat & getHomography() const {return h_;}

protected:
	virtual void run()
	{
		//QTime time;
		//time.start();

		std::vector<cv::Point2f> mpts_1(matches_->size());
		std::vector<cv::Point2f> mpts_2(matches_->size());
		indexesA_.resize(matches_->size());
		indexesB_.resize(matches_->size());

		int j=0;
		for(QMultiMap<int, int>::const_iterator iter = matches_->begin(); iter!=matches_->end(); ++iter)
		{
			mpts_1[j] = kptsA_->at(iter.key()).pt;
			indexesA_[j] = iter.key();
			mpts_2[j] = kptsB_->at(iter.value()).pt;
			indexesB_[j] = iter.value();
			++j;
		}

		if((int)mpts_1.size() >= Settings::getHomography_minimumInliers())
		{
			h_ = findHomography(mpts_1,
					mpts_2,
					Settings::getHomographyMethod(),
					Settings::getHomography_ransacReprojThr(),
					outlierMask_);

			for(unsigned int k=0; k<mpts_1.size();++k)
			{
				if(outlierMask_.at(k))
				{
					inliers_.insert(indexesA_[k], indexesB_[k]);
				}
				else
				{
					outliers_.insert(indexesA_[k], indexesB_[k]);
				}
			}

			// ignore homography when all features are inliers
			if(inliers_.size() == (int)outlierMask_.size() && !h_.empty())
			{
				if(Settings::getHomography_ignoreWhenAllInliers() || cv::countNonZero(h_) < 1)
				{
					h_ = cv::Mat();
				}
			}
		}

		//UINFO("Homography Object %d time=%d ms", objectIndex_, time.elapsed());
	}
private:
	const QMultiMap<int, int> * matches_;
	int objectId_;
	const std::vector<cv::KeyPoint> * kptsA_;
	const std::vector<cv::KeyPoint> * kptsB_;

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
	QMultiMap<int,QPair<QRect,QTransform> > objects;
	this->detect(image, objects);
	if(objects.size() > 0 || Settings::getGeneral_sendNoObjDetectedEvents())
	{
		Q_EMIT objectsFound(objects);
	}

	if(objects.size() > 1)
	{
		UINFO("(%s) %d objects detected! (%d ms)",
				QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str(),
				(int)objects.size(),
				time.elapsed());
	}
	else if(objects.size() == 1)
	{
		UINFO("(%s) Object %d detected! (%d ms)",
				QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str(),
				(int)objects.begin().key(),
				time.elapsed());
	}
	else if(Settings::getGeneral_sendNoObjDetectedEvents())
	{
		UINFO("(%s) No objects detected. (%d ms)",
				QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str(),
				time.elapsed());
	}
}

bool FindObject::detect(const cv::Mat & image, QMultiMap<int,QPair<QRect,QTransform> > & objectsDetected)
{
	QTime totalTime;
	totalTime.start();

	// reset statistics
	objectsDetected_.clear();
	timeStamps_.clear();
	sceneKeypoints_.clear();
	sceneDescriptors_ = cv::Mat();
	sceneWords_.clear();
	matches_.clear();
	inliers_.clear();
	outliers_.clear();
	minMatchedDistance_ = -1.0f;
	maxMatchedDistance_ = -1.0f;

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

		QTime time;
		time.start();

		// EXTRACT KEYPOINTS
		detector_->detect(grayscaleImg, sceneKeypoints_);
		timeStamps_.insert(kTimeKeypointDetection, time.restart());

		if(sceneKeypoints_.size())
		{
			int maxFeatures = Settings::getFeature2D_3MaxFeatures();
			if(maxFeatures > 0 && (int)sceneKeypoints_.size() > maxFeatures)
			{
				sceneKeypoints_ = limitKeypoints(sceneKeypoints_, maxFeatures);
			}

			// EXTRACT DESCRIPTORS
			extractor_->compute(grayscaleImg, sceneKeypoints_, sceneDescriptors_);
			if((int)sceneKeypoints_.size() != sceneDescriptors_.rows)
			{
				UERROR("kpt=%d != descriptors=%d", (int)sceneKeypoints_.size(), sceneDescriptors_.rows);
			}
		}
		else
		{
			UWARN("no features detected !?!");
		}
		timeStamps_.insert(kTimeDescriptorExtraction, time.restart());

		bool consistentNNData = (vocabulary_->size()!=0 && vocabulary_->wordToObjects().begin().value()!=-1 && Settings::getGeneral_invertedSearch()) ||
								((vocabulary_->size()==0 || vocabulary_->wordToObjects().begin().value()==-1) && !Settings::getGeneral_invertedSearch());

		// COMPARE
		if(!objectsDescriptors_.empty() &&
			sceneKeypoints_.size() &&
		   consistentNNData &&
		   objectsDescriptors_.begin().value().cols == sceneDescriptors_.cols &&
		   objectsDescriptors_.begin().value().type() == sceneDescriptors_.type()) // binary descriptor issue, if the dataTree is not yet updated with modified settings
		{
			success = true;

			QMultiMap<int, int> words;

			if(!Settings::getGeneral_invertedSearch())
			{
				// CREATE INDEX for the scene
				vocabulary_->clear();
				words = vocabulary_->addWords(sceneDescriptors_, -1, Settings::getGeneral_vocabularyIncremental());
				if(!Settings::getGeneral_vocabularyIncremental())
				{
					vocabulary_->update();
				}
				timeStamps_.insert(kTimeIndexing, time.restart());
			}

			for(QMap<int, ObjSignature*>::iterator iter=objects_.begin(); iter!=objects_.end(); ++iter)
			{
				matches_.insert(iter.key(), QMultiMap<int, int>());
			}

			if(Settings::getGeneral_invertedSearch() || Settings::getGeneral_threads() == 1)
			{
				cv::Mat results;
				cv::Mat dists;
				// DO NEAREST NEIGHBOR
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
					results = cv::Mat(sceneDescriptors_.rows, k, CV_32SC1); // results index
					dists = cv::Mat(sceneDescriptors_.rows, k, CV_32FC1); // Distance results are CV_32FC1
					vocabulary_->search(sceneDescriptors_, results, dists, k);
				}

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

					if(matched)
					{
						if(Settings::getGeneral_invertedSearch())
						{
							int wordId = results.at<int>(i,0);
							QList<int> objIds = vocabulary_->wordToObjects().values(wordId);
							for(int j=0; j<objIds.size(); ++j)
							{
								// just add unique matches
								if(vocabulary_->wordToObjects().count(wordId, objIds[j]) == 1)
								{
									matches_.find(objIds[j]).value().insert(objects_.value(objIds[j])->words().value(wordId), i);
								}
							}
						}
						else
						{
							QMap<int, int>::iterator iter = dataRange_.lowerBound(i);
							int objectId = iter.value();
							int fisrtObjectDescriptorIndex = (iter == dataRange_.begin())?0:(--iter).key()+1;
							int objectDescriptorIndex = i - fisrtObjectDescriptorIndex;

							int wordId = results.at<int>(i,0);
							if(words.count(wordId) == 1)
							{
								matches_.find(objectId).value().insert(objectDescriptorIndex, words.value(wordId));
							}
						}
					}
				}
			}
			else
			{
				//multi-threaded, match objects to scene
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
						matches_[threads[k]->getObjectId()] = threads[k]->getMatches();

						if(minMatchedDistance_ == -1 || minMatchedDistance_ > threads[k]->getMinMatchedDistance())
						{
							minMatchedDistance_ = threads[k]->getMinMatchedDistance();
						}
						if(maxMatchedDistance_ == -1 || maxMatchedDistance_ < threads[k]->getMaxMatchedDistance())
						{
							maxMatchedDistance_ = threads[k]->getMaxMatchedDistance();
						}
						delete threads[k];
					}

				}
			}

			timeStamps_.insert(kTimeMatching, time.restart());

			// Homographies
			if(Settings::getHomography_homographyComputed())
			{
				// HOMOGRAPHY
				int threadCounts = Settings::getGeneral_threads();
				if(threadCounts == 0)
				{
					threadCounts = matches_.size();
				}
				QList<int> matchesId = matches_.keys();
				QList<QMultiMap<int, int> > matchesList = matches_.values();
				for(int i=0; i<matchesList.size(); i+=threadCounts)
				{
					QVector<HomographyThread*> threads;

					for(int k=i; k<i+threadCounts && k<matchesList.size(); ++k)
					{
						int objectId = matchesId[k];
						threads.push_back(new HomographyThread(&matchesList[k], objectId, &objects_.value(objectId)->keypoints(), &sceneKeypoints_));
						threads.back()->start();
					}

					for(int j=0; j<threads.size(); ++j)
					{
						threads[j]->wait();

						int id = threads[j]->getObjectId();

						if(!threads[j]->getHomography().empty())
						{
							if(threads[j]->getInliers().size() >= Settings::getHomography_minimumInliers())
							{
								const cv::Mat & H = threads[j]->getHomography();
								QTransform hTransform(
									H.at<double>(0,0), H.at<double>(1,0), H.at<double>(2,0),
									H.at<double>(0,1), H.at<double>(1,1), H.at<double>(2,1),
									H.at<double>(0,2), H.at<double>(1,2), H.at<double>(2,2));

								int distance = Settings::getGeneral_multiDetectionRadius(); // in pixels
								if(Settings::getGeneral_multiDetection())
								{
									// Get the outliers and recompute homography with them
									matchesList.push_back(threads[j]->getOutliers());
									matchesId.push_back(id);

									// compute distance from previous added same objects...
									QMultiMap<int, QPair<QRect, QTransform> >::iterator objIter = objectsDetected.find(id);
									for(;objIter!=objectsDetected.end() && objIter.key() == id; ++objIter)
									{
										qreal dx = objIter.value().second.m31() - hTransform.m31();
										qreal dy = objIter.value().second.m32() - hTransform.m32();
										int d = (int)sqrt(dx*dx + dy*dy);
										if(d < distance)
										{
											distance = d;
										}
									}
								}

								if(distance >= Settings::getGeneral_multiDetectionRadius())
								{
									QRect rect = objects_.value(id)->rect();
									objectsDetected.insert(id, QPair<QRect, QTransform>(rect, hTransform));
									inliers_.insert(id, threads[j]->getInliers());
									outliers_.insert(id, threads[j]->getOutliers());
								}
								else
								{
									rejectedInliers_.insert(id, threads[j]->getInliers());
									rejectedOutliers_.insert(id, threads[j]->getOutliers());
								}
							}
							else
							{
								rejectedInliers_.insert(id, threads[j]->getInliers());
								rejectedOutliers_.insert(id, threads[j]->getOutliers());
							}
						}
						else
						{
							rejectedInliers_.insert(id, threads[j]->getInliers());
							rejectedOutliers_.insert(id, threads[j]->getOutliers());
						}
					}
				}
				timeStamps_.insert(kTimeHomography, time.restart());
			}
		}
		else if(!objectsDescriptors_.empty() && sceneKeypoints_.size())
		{
			UWARN("Cannot search, objects must be updated");
		}
	}

	objectsDetected_ = objectsDetected;
	timeStamps_.insert(kTimeTotal, totalTime.elapsed());

	return success;
}
