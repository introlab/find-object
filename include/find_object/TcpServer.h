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

#ifndef TCPSERVER_H_
#define TCPSERVER_H_

#include "find_object/FindObjectExp.h" // DLL export/import defines

#include "find_object/DetectionInfo.h"
#include <opencv2/opencv.hpp>

#include <QtNetwork/QTcpServer>

namespace find_object {

class QNetworkSession;

class FINDOBJECT_EXP TcpServer : public QTcpServer
{
	Q_OBJECT

public:
	enum Service {
		kAddObject,   // id fileName imageSize image
		kRemoveObject // id
	};

public:
	TcpServer(quint16 port = 0, QObject * parent = 0);

	QHostAddress getHostAddress() const;
	quint16 getPort() const;

public Q_SLOTS:
	void publishDetectionInfo(const find_object::DetectionInfo & info);

private Q_SLOTS:
	void addClient();
	void readReceivedData();
	void displayError(QAbstractSocket::SocketError socketError);
	void connectionLost();

Q_SIGNALS:
	void addObject(const cv::Mat &, int, const QString &);
	void removeObject(int);

private:
	QMap<int, quint64> blockSizes_;
};

} // namespace find_object

#endif /* TCPSERVER_H_ */
