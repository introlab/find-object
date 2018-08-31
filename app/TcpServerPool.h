/*
 * TcpServerPool.h
 *
 *  Created on: Nov 29, 2015
 *      Author: mathieu
 */

#ifndef TCPSERVERPOOL_H_
#define TCPSERVERPOOL_H_

#include <find_object/FindObject.h>
#include <find_object/TcpServer.h>
#include <find_object/utilite/ULogger.h>
#include <QtCore/QThread>
#include <QtCore/QSemaphore>

class FindObjectWorker : public QObject
{
	Q_OBJECT;

public:
	FindObjectWorker(
			find_object::FindObject * sharedFindObject,
			QSemaphore * sharedSemaphore,
			int maxSemaphoreResources,
			QObject * parent = 0) :
		QObject(parent),
		sharedFindObject_(sharedFindObject),
		sharedSemaphore_(sharedSemaphore),
		maxSemaphoreResources_(maxSemaphoreResources)
	{
		UASSERT(sharedFindObject != 0);
		UASSERT(sharedSemaphore != 0);
		UASSERT(maxSemaphoreResources > 0);
	}

public Q_SLOTS:
	void detect(const cv::Mat & image)
	{
		sharedSemaphore_->acquire(1);
		UINFO("Thread %p detecting...", (void *)this->thread());
		find_object::DetectionInfo info;
		sharedFindObject_->detect(image, info);
		Q_EMIT objectsFound(info);
		sharedSemaphore_->release(1);
	}

	void addObjectAndUpdate(const cv::Mat & image, int id, const QString & filePath)
	{
		//block everyone!
		sharedSemaphore_->acquire(maxSemaphoreResources_);
		UINFO("Thread %p adding object %d (%s)...", (void *)this->thread(), id, filePath.toStdString().c_str());
		sharedFindObject_->addObjectAndUpdate(image, id, filePath);
		sharedSemaphore_->release(maxSemaphoreResources_);
	}
	void removeObjectAndUpdate(int id)
	{
		//block everyone!
		sharedSemaphore_->acquire(maxSemaphoreResources_);
		UINFO("Thread %p removing object %d...", (void *)this->thread(), id);
		sharedFindObject_->removeObjectAndUpdate(id);
		sharedSemaphore_->release(maxSemaphoreResources_);
	}

Q_SIGNALS:
	void objectsFound(const find_object::DetectionInfo &);

private:
	find_object::FindObject * sharedFindObject_; //shared findobject
	QSemaphore * sharedSemaphore_;
	int maxSemaphoreResources_;
};

class TcpServerPool : public QObject
{
	Q_OBJECT;
public:
	TcpServerPool(find_object::FindObject * sharedFindObject, int threads, int port) :
		sharedSemaphore_(threads)
	{
		UASSERT(sharedFindObject != 0);
		UASSERT(threads>=1);

		qRegisterMetaType<cv::Mat>("cv::Mat");

		threadPool_.resize(threads);
		for(int i=0; i<threads; ++i)
		{
			find_object::TcpServer * tcpServer =  new find_object::TcpServer(port!=0?port++:0);
			UINFO("TcpServer set on port: %d (IP=%s)",
					tcpServer->getPort(),
					tcpServer->getHostAddress().toString().toStdString().c_str());

			threadPool_[i] = new QThread(this);
			FindObjectWorker * worker = new FindObjectWorker(sharedFindObject, &sharedSemaphore_, threads);

			tcpServer->moveToThread(threadPool_[i]);
			 worker->moveToThread(threadPool_[i]);
			 connect(threadPool_[i], SIGNAL(finished()), tcpServer, SLOT(deleteLater()));
			connect(threadPool_[i], SIGNAL(finished()), worker, SLOT(deleteLater()));

			// connect stuff:
			QObject::connect(worker, SIGNAL(objectsFound(find_object::DetectionInfo)), tcpServer, SLOT(publishDetectionInfo(find_object::DetectionInfo)));
			QObject::connect(tcpServer, SIGNAL(detectObject(const cv::Mat &)), worker, SLOT(detect(const cv::Mat &)));
			QObject::connect(tcpServer, SIGNAL(addObject(const cv::Mat &, int, const QString &)), worker, SLOT(addObjectAndUpdate(const cv::Mat &, int, const QString &)));
			QObject::connect(tcpServer, SIGNAL(removeObject(int)), worker, SLOT(removeObjectAndUpdate(int)));
			threadPool_[i]->start();
		}
	}

	virtual ~TcpServerPool()
	{
		for(int i=0; i<threadPool_.size(); ++i)
		{
			threadPool_[i]->quit();
			threadPool_[i]->wait();
		}
	}

private:
	QVector<QThread*> threadPool_;
	QSemaphore sharedSemaphore_;
};



#endif /* TCPSERVERPOOL_H_ */
