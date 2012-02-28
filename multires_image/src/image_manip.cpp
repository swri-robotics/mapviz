/*
* ImageManip.h
* 
* Copyright (C) 2010 Southwest Research Institute <malban@swri.org>
*
* Created on: Mar 01, 2010
*     Author: Marc Alban
*/

#include <multires_image/image_manip.h>

// C++ standard libraries
#include <iostream>
#include <exception>

// QT libraries
#include <QStringList>
#include <QProcess>
#include <QByteArray>

namespace multires_image
{

bool ImageManip::knownPlatform = false;
bool ImageManip::isWindows = false;
bool ImageManip::isLinux = false;

bool ImageManip::ResizeImage(QString src, QString dst, int percent)
{
	if (knownPlatform)
	{
		if (isWindows)
		{
			return ResizeImageWindows(src, dst, percent);
		}
		if (isLinux)
		{
			return ResizeImageLinux(src, dst, percent);
		}
	}

	return false;
}

bool ImageManip::ResizeImageWindows(QString src, QString dst, int percent)
{
	return true;
}

bool ImageManip::ResizeImageLinux(QString src, QString dst, int percent)
{
	return true;
}

bool ImageManip::TileImage(QString src, QString dst, int size)
{
	return true;
}

bool ImageManip::TileImageWindows(QString src, QString dst, int size)
{
	return true;
}

bool ImageManip::TileImageLinux(QString src, QString dst, int size)
{
	return true;
}

bool ImageManip::GetDimensions(QString path, int& width, int& height)
{
	// Try windows version.
	if (!GetDimensionsWindows(path, width, height))
	{
		if (!GetDimensionsLinux(path, width, height))
		{
			return false;	
		}
	}
	
	return true;
}

bool ImageManip::GetDimensionsWindows(QString path, int& width, int& height)
{
	width = 0;
	height = 0;

	bool ok = false;

	try
	{
		QStringList arguments;
		arguments << "-format";
		arguments << "%w %h";
		arguments << path;
		
		QProcess process;
		process.setProcessChannelMode(QProcess::MergedChannels);
		process.start("identify.exe", arguments);
		process.waitForFinished(5000);

		QString data(process.readAllStandardOutput());
		QStringList values = data.split(' ', QString::SkipEmptyParts);

		if (values.size() == 2)
		{
			width = values.at(0).toInt(&ok, 10);
			if (ok)
			{
				height = values.at(1).toInt(&ok, 10);
			}
		}
	}
	catch (std::exception& e)
	{
		std::cout << "An exception occured reading image dimensions: " << e.what() << std::endl;
		return false;
	}	

	if (!ok)
	{
		return false;
	}

	knownPlatform = true;
	isWindows = true;
	return true;
}

bool ImageManip::GetDimensionsLinux(QString path, int& width, int& height)
{
	return false;
}

}
