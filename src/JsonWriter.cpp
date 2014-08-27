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

#include "find_object/JsonWriter.h"
#include "find_object/utilite/ULogger.h"

#include <QtCore/QFile>
#include <QtCore/QTextStream>

#include "json/json.h"

namespace find_object {

void JsonWriter::write(const DetectionInfo & info, const QString & path)
{
	if(!path.isEmpty())
	{
		Json::Value root;

		if(info.objDetected_.size())
		{
			Json::Value detections;

			QMultiMap<int, int>::const_iterator iterInliers = info.objDetectedInliersCount_.constBegin();
			QMultiMap<int, int>::const_iterator iterOutliers = info.objDetectedOutliersCount_.constBegin();
			QMultiMap<int, QSize>::const_iterator iterSizes = info.objDetectedSizes_.constBegin();
			QMultiMap<int, QString>::const_iterator iterFilenames = info.objDetectedFilenames_.constBegin();
			for(QMultiMap<int, QTransform>::const_iterator iter = info.objDetected_.constBegin(); iter!= info.objDetected_.end();)
			{
				char index = 'a';
				int id = iter.key();
				while(iter != info.objDetected_.constEnd() && id == iter.key())
				{
					QString name = QString("object_%1%2").arg(id).arg(info.objDetected_.count(id)>1?QString(index++):"");
					detections.append(name.toStdString());

					Json::Value homography;
					homography.append(iter.value().m11());
					homography.append(iter.value().m12());
					homography.append(iter.value().m13());
					homography.append(iter.value().m21());
					homography.append(iter.value().m22());
					homography.append(iter.value().m23());
					homography.append(iter.value().m31());  // dx
					homography.append(iter.value().m32());  // dy
					homography.append(iter.value().m33());
					root[name.toStdString()]["width"] = iterSizes.value().width();
					root[name.toStdString()]["height"] = iterSizes.value().height();
					root[name.toStdString()]["homography"] = homography;
					root[name.toStdString()]["inliers"] = iterInliers.value();
					root[name.toStdString()]["outliers"] = iterOutliers.value();
					root[name.toStdString()]["filename"] = iterFilenames.value().toStdString();

					++iter;
					++iterInliers;
					++iterOutliers;
					++iterSizes;
					++iterFilenames;
				}
			}
			root["objects"] = detections;
		}

		if(info.matches_.size())
		{
			Json::Value matchesValues;
			const QMap<int, QMultiMap<int, int> > & matches = info.matches_;
			for(QMap<int, QMultiMap<int, int> >::const_iterator iter = matches.constBegin();
				iter != matches.end();
				++iter)
			{
				QString name = QString("matches_%1").arg(iter.key());
				root[name.toStdString()] = iter.value().size();
				matchesValues.append(name.toStdString());
			}
			root["matches"] = matchesValues;
		}

		// write in a nice readible way
		Json::StyledWriter styledWriter;
		//std::cout << styledWriter.write(root);
		QFile file(path);
		file.open(QIODevice::WriteOnly | QIODevice::Text);
		QTextStream out(&file);
		out << styledWriter.write(root).c_str();
		file.close();
	}
}

} // namespace find_object
